//----------------------------------------------//
//CÓDIGO MICROCONTROLADOR ESP32
//Código general del microcontrolador ESP32 usando micro-ROS para el control del Robot.
//Taller #2 ROBÓTICA
//----------------------------------------------//

//Librerías principales
#include <Arduino.h>
#include <micro_ros_platformio.h>

//Librerías micro-ROS
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

//Tipos de mensajes
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>


//Fallback para asegurarse que el ESP32 esté conectado mediante Serial.
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif


//Definición del publisher y del subscriber
rcl_publisher_t positionpublisher;
rclc_executor_t executor_pub;

rcl_subscription_t cmdVel_subscriber;
rclc_executor_t executor_sub;

//Definición de variables de los topicos a usar
const char * position_topic = "/robot_position";
const char * cmdVel_topic = "/robot_cmdVel";

//Variables de mensaje tipo Twist
geometry_msgs__msg__Twist position_msg;
geometry_msgs__msg__Twist cmdVel_msg;


//Parámetros necesarios para la conexión con el Raspberry.
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//No tengo la más remota idea
#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

// Manejo de errores
void error_loop() {
  while(1) {
    delay(100);
  }
}


//Variables de movimiento

float Vmax = 7.3;
float CTE_Motor1 = 149; //95; //medir esta constante experimentalmente
float CTE_Motor2 = 150; //104; //medir esta constante experimentalmente
float kp1 = 0.02;
float ki1 = 0.00015 ;
float kd1 = 0;
float kp2 = 0.02;
float ki2 = 0.00015 ;
float kd2 = 0;
//float RPM_max = 220; //CHANGED (12 V)
float RPM_max = 66; //CHANGED (7.3 V)

//timer for esp32
hw_timer_t * timere = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool timerFlag = false;

void IRAM_ATTR onTimer() {
  // Code to be executed when the timer triggers an interrupt
  timerFlag = true;
}

unsigned long t;
unsigned long t_prev = 0;

const byte Encoder1PinA = 34;
const byte Encoder1PinB = 35;
const byte Encoder2PinA = 18;
const byte Encoder2PinB = 19;

volatile long EncoderCount1 = 0;
volatile long EncoderCount2 = 0;

const byte Motor1Out1 = 33;
const byte Motor1Out2 = 25;
const byte Motor1PWM = 32;
const byte Motor2Out1 = 26;
const byte Motor2Out2 = 27;
const byte Motor2PWM = 14;


float Theta1, RPM1, RPM1_target;
float Theta1_prev = 0;
float Theta2, RPM2, RPM2_target;
float Theta2_prev = 0;
int dt;


#define pi 3.14159265
float Vmin = -Vmax;
float V1 = 0.1, V2 = 0.1;
float e1, e1_prev = 0, inte1, inte1_prev = 0;
float e2, e2_prev = 0, inte2, inte2_prev = 0;

float r_llanta=6.73/2;
float l_eje=12.6;

float v_lineal, v_angular;
float v_lineal_max=(RPM_max*(pi/30))*r_llanta;
float v_angular_max=2*v_lineal_max/l_eje;

//MPU
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//AAA

float norma = 0;

float pos_x = 0;
float pos_y = 0;


//Callback constante para el publisher
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {

  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    RCSOFTCHECK(rcl_publish(&positionpublisher, &position_msg, NULL));
    //position_msg.linear.x = RPM1;
    //position_msg.linear.y = RPM2;
    position_msg.linear.x = pos_x;
    position_msg.linear.y = pos_y;
  }

}

//Callback para el subscriber

void cmdVel_callback(const void * msgin)
{
  geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *) msgin;
  v_lineal=msg->linear.x;
  v_angular=msg->angular.z;
}

//DE ACÁ PARA ABAJO FUNCIONES DE MOVIMIENTO

void ISR_Encoder1() {
  bool PinB = digitalRead(Encoder1PinB);
  bool PinA = digitalRead(Encoder1PinA);

  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderCount1++;
    }
    else {
      EncoderCount1--;
    }
  }

  else {
    if (PinA == HIGH) {
      EncoderCount1--;
    }
    else {
      EncoderCount1++;
    }
  }
}

void ISR_Encoder2() {
  bool PinB = digitalRead(Encoder2PinB);
  bool PinA = digitalRead(Encoder2PinA);

  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderCount2++;
    }
    else {
      EncoderCount2--;
    }
  }

  else {
    if (PinA == HIGH) {
      EncoderCount2--;
    }
    else {
      EncoderCount2++;
    }
  }
}


float sign(float x) {
  if (x > 0) {
    return 1;
  } else if (x < 0) {
    return -1;
  } else {
    return 0;
  }
}

//***Motor Driver Functions*****
void WriteDriverVoltage(float V, float Vmax, int DirPin1, int DirPin2, int PWMPin) {
  int PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 255) {
    PWMval = 255;
  }
  if (V > 0) {
    digitalWrite(DirPin1, HIGH);
    digitalWrite(DirPin2, LOW);
  }
  else if (V < 0) {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, HIGH);
  }
  else {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, LOW);
  }
  analogWrite(PWMPin, PWMval);
}

void Velocity_to_RPM(float v_lin, float v_ang){
  //v_lin en cm/s
  //v_ang en rad/s

  v_lin=-v_lin; //en el cambio de base 6 se invirtió la orientación

  if(abs(v_lin)>v_lineal_max){
    v_lin=v_lineal_max*sign(v_lin);
  } 
  if(abs(v_ang)>v_angular_max){
    v_ang=v_angular_max*sign(v_ang);
  }
  
  //la llanta 1 sigue al motor 1, la llanta 2 invierte el signo del motor 2
  if(v_lin!=0 && v_ang==0){
    RPM1_target=(v_lin/r_llanta)*(30/pi);
    RPM2_target=-(v_lin/r_llanta)*(30/pi);
  }

  //motor 1 es llanta izquierda motor 2 es llanta derecha
  //formula v_ang=2*v_lin/l
  //v_lin=v_ang*l/2
  else if(v_lin==0 && v_ang!=0){ //gira hacia un lado
    float v_lin_prov=v_ang*l_eje/2; //v_ang+/- da v_lin_prov +/-
    RPM1_target=-(v_lin_prov/r_llanta)*(30/pi); //lanta 1 hacia atras/delante
    RPM2_target=-(v_lin_prov/r_llanta)*(30/pi); //lanta 2 hacia delante/atras
  }
  else{
    RPM1_target=0;
    RPM2_target=0;
  }
}

void setup() {
  // Inizialización del serial
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Inizialización del Allocator
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Creación del nodo del ESP32
  RCCHECK(rclc_node_init_default(&node, "robot_controller", "", &support));

  // Creación del Publisher
  RCCHECK(rclc_publisher_init_default(
    &positionpublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    position_topic));

  // Creación del Subscriber

  RCCHECK(rclc_subscription_init_default(
      &cmdVel_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      cmdVel_topic));

  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &cmdVel_subscriber, &cmdVel_msg, &cmdVel_callback, ALWAYS));

  //DE ACÁ PARA ABAJO MOVIMIENTO

  pinMode(Encoder1PinA, INPUT_PULLUP);
  pinMode(Encoder1PinB, INPUT_PULLUP);
  pinMode(Encoder2PinA, INPUT_PULLUP);
  pinMode(Encoder2PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Encoder1PinA), ISR_Encoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder2PinA), ISR_Encoder2, CHANGE);
  pinMode(Motor1Out1, OUTPUT);
  pinMode(Motor1Out2, OUTPUT);
  pinMode(Motor2Out1, OUTPUT);
  pinMode(Motor2Out2, OUTPUT);
  pinMode(Motor1PWM, OUTPUT);
  pinMode(Motor2PWM, OUTPUT);


  //timer interrupt in esp32
  portENTER_CRITICAL(&timerMux);
  // Initialize Timer
  timere = timerBegin(0, 80, true); // Timer 0, Prescaler 80 (divided by 80), Count up
  // Attach the interrupt service routine
  timerAttachInterrupt(timere, &onTimer, true);
  // Set the alarm value (equivalent to OCR1A)
  timerAlarmWrite(timere, 10000, true); // Alarm every 12500 counts (equivalent to 12500/80 microseconds)
  // Enable the timer interrupt
  timerAlarmEnable(timere);
  // Enable interrupts globally after configuration
  portEXIT_CRITICAL(&timerMux);


  v_lineal=0.00;
  v_angular=0.00;





  //MPU
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif


    //while (!Serial); // wait for Leonardo enumeration, others continue immediately
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    //RESULTADO INICIALIZACIÓN:
    //-2906.00000,	1353.00000,	1318.00000,	-34.00000,	-50.00000,	10.00000

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-34.00000);
    mpu.setYGyroOffset(-50.00000);
    mpu.setZGyroOffset(10.00000);
    mpu.setXAccelOffset(-2906.00000); // 1688 factory default for my test chip
    mpu.setYAccelOffset(1353.00000); // 1688 factory default for my test chip
    mpu.setZAccelOffset(1318.00000); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));

    }


}

void loop() {

  if (timerFlag){

    RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
    RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
    
    t = millis();
    dt = (t - t_prev);

    Theta1 = EncoderCount1 / CTE_Motor1; //CHANGED
    Theta2 = EncoderCount2 / CTE_Motor2; //CHANGED
    
    //RPM1_target = ?
    //RPM2_target = ?
    Velocity_to_RPM(v_lineal,v_angular);
    
    RPM1 = (Theta1 - Theta1_prev) / (dt / 1000.0) * 60;
    RPM2 = (Theta2 - Theta2_prev) / (dt / 1000.0) * 60;

    e1 = RPM1_target - RPM1;
    e2 = RPM2_target - RPM2;

    inte1 = inte1_prev + (dt * (e1 + e1_prev) / 2);
    inte2 = inte2_prev + (dt * (e2 + e2_prev) / 2);

    V1 = kp1 * e1 + ki1 * inte1 + (kd1 * (e1 - e1_prev) / dt) ;
    V2 = kp2 * e2 + ki2 * inte2 + (kd2 * (e2 - e2_prev) / dt) ;
    

    //Anti Wind-Up
    if (V1 > Vmax) {
      V1 = Vmax;
      inte1 = inte1_prev;
    }
    if (V1 < Vmin) {
      V1 = Vmin;
      inte1 = inte1_prev;
    }
    if (V2 > Vmax) {
      V2 = Vmax;
      inte2 = inte2_prev;
    }
    if (V2 < Vmin) {
      V2 = Vmin;
      inte2 = inte2_prev;
    }



    WriteDriverVoltage(V1, Vmax, Motor1Out1,Motor1Out2,Motor1PWM);
    WriteDriverVoltage(V2, Vmax, Motor2Out1,Motor2Out2,Motor2PWM);

    Theta1_prev = Theta1;
    Theta2_prev = Theta2;
    t_prev = t;
    inte1_prev = inte1;
    inte2_prev = inte2;
    e1_prev = e1;
    e2_prev = e2;





    //MPU
    if (dmpReady){ 
    // read a packet from FIFO
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      }
    }

    norma = RPM2 * 2*pi*r_llanta *dt /(60* 1000);

    pos_x += norma * sin(ypr[0]);
    pos_y += norma * cos(ypr[0]);

    timerFlag = false; // Reset the flag
  }

}
