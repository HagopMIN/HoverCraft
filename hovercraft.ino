//Libraries 
#include "hovercraft.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


// ================================================================
// ===                 IMU Variables                            ===
// ================================================================
MPU6050 mpu;
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define PWM_LED_PIN 11
bool blinkState = false;

#define DEBUG
#define SERVO_MOTOR_DEMO

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



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                 Important Variables                      ===
// ================================================================
// Variables for integration
unsigned long previousMillis = 0; // To store the last time the measurement was taken
float velocityX = 0.0; // Velocity in the x-axis
float positionX = 0.0; // Position in the x-axis

// High-pass filter variables
float alpha = 0.5;
float filteredAx = 0.0;
float prevAx = 0.0; // Previous acceleration value
float yd ;


// ================================================================
// ===                 STATE CONTROL                            ===
// ================================================================
int h_state =   lift;


// ================================================================
// ===                    TIMERS                                ===
// ================================================================
noDelay L_speedControl(1);
noDelay T_speedControl(1);


// ================================================================
// ===                    SpeedControl                          ===
// ================================================================
int L_currentSpeed = 0;         // Current speed of the fan
int T_currentSpeed = 0;         // Current speed of the fan


// ================================================================
// ===                      Servo                               ===
// ================================================================
int pos;  // position of servo (range between 0 and 180; 90 is the neutral position)
Servo Servo_motor; 

// ================================================================
// ===                      UltraSonic Sensor                   ===
// ================================================================
int FrontDistance; 
int Front_Reached; 

int SideSensorDistance;
int LeftTurn_Reached;
int RightTurn_Reached; 


// NewPing setup of pins and maximum distance.
NewPing frontsonar(L_TRIGGER_PIN, L_ECHO_PIN, MAX_DISTANCE);
NewPing leftsonar(F_TRIGGER_PIN, F_ECHO_PIN, MAX_DISTANCE);

void setup() {
  //MUST be initialzied before everythin else.
  Serial.begin(BAUDRATE);
  //IMU Init:
  MPU_6050_Init();
  //FAN INIT
  initialize_lift_fan();
  #ifdef DEBUG
  Serial.println("Lift_fan: Ready");
  #endif 
  initialize_throttle_fan();
  #ifdef DEBUG
  Serial.println("Throttle_fan: Ready");
  #endif 

  //init Servo
  Servo_motor.attach(ServoPin);  // Attach the servo to pin 9
  Servo_motor.write(NinetyDeg); //Start position always at 90 

}

void loop() {

    switch (h_state){
      case lift:
      if(L_speedControl.update()){
          L_currentSpeed = L_currentSpeed + 10;
          T_currentSpeed++; 
          if(L_currentSpeed >= L_MaxSpeed && T_currentSpeed >= T_SixtySpeed){
            L_currentSpeed = L_MaxSpeed;
            T_currentSpeed = T_SixtySpeed;
            h_state = moveForward;
          }
          #ifdef DEBUG
          Serial.println("L");
          #endif 
      }
      
      set_lift_speed(L_currentSpeed);
      set_throttle_speed(T_currentSpeed);
      break;

      case moveForward:
          MPU_6050_ActiveCorrection();
          if(frontwallreached()){
            h_state = turn;
            set_throttle_speed(T_STOP);
          }
          #ifdef DEBUG
          Serial.println("MF");
          #endif 
      break;
      case turn:
          h_state = sideWallCheck();
          break;
      case turnRight:
          set_throttle_speed(255);
          Servo_motor.write(165);
          delay(2000);
          Servo_motor.write(85);
          delay(2000);
          Servo_motor.write(165);
          h_state = dontn;
          
          
          #ifdef DEBUG
          Serial.println("TR");
          #endif 
      break;

      case turnLeft: 
          set_throttle_speed(60);
          Servo_motor.write(45);
          #ifdef DEBUG
          Serial.println("TL");
          #endif 
          h_state = dontn;
      break;
      case idle:
        Servo_motor.write(90);
        set_throttle_speed(150);
        
        h_state = ex;
      break;
      case ex:
        Servo_motor.write(0);
        set_throttle_speed(255);
      
        h_state = s;
      break;
      case s:
        Servo_motor.write(90);
        set_throttle_speed(0);
      break;
      case dontn: 
      break;

    }
   

} 

void MPU_6050_Init(void){
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    //Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device

    #ifdef DEBUG
    Serial.println(F("Initializing I2C devices..."));
    #endif
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    #ifdef DEBUG
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    #endif


    // wait for ready
    #ifdef DEBUG
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    #endif

    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(0); //inital value 220 
    mpu.setYGyroOffset(0); //inital Value 76 
    mpu.setZGyroOffset(-25); //inital value -85 
    mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(10);
        mpu.CalibrateGyro(10);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        #ifdef DEBUG
        Serial.println(F("Enabling DMP..."));
        #endif
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        #ifdef DEBUG
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        #endif
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        #ifdef DEBUG
        Serial.println(F(")..."));
        #endif
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        #ifdef DEBUG
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        #endif
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        #ifdef DEBUG
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        #endif
    }
  
}

void MPU_6050_ActiveCorrection(void){
if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      #ifdef SERVO_MOTOR_DEMO
           mpu.dmpGetQuaternion(&q, fifoBuffer);
            //mpu.dmpGetGravity(&gravity, &q);
            //mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            float q0 = q.w;
            float q1 = q.x; 
            float q2 = q.y;
            float q3 = q.z;

            float yr = -atan2(-2 * q1 * q2 + 2 * q0 * q3, q2 * q2 - q3 * q3 - q1 * q1 + q0 * q0 ); 
            float yd = yr * 180/M_PI;
            #ifdef DEBUG
            Serial.print("yaw\t");
            Serial.println(yd);
            #endif

            if(yd < 85 && yd > (-85)){
              #ifdef DEBUG
              Serial.println("Servo Move");
              #endif
              
              pos = 85 + (-yd);
              Servo_motor.write(pos);
            }else{
              #ifdef DEBUG
              Serial.println("out of range");
              #endif
              
            }

            delay(100); 
        #endif
   }
}

bool frontwallreached(void){
  FrontDistance = frontsonar.ping_cm();
  if(FrontDistance <= 25){
    Front_Reached++; 
  }
  if(Front_Reached >= SAMPLESIZE){
    #ifdef DEBUG
    Serial.println("out of range");
    #endif
    return true;
  }else{
    return false;
  }

}


int sideWallCheck(void){
  SideSensorDistance = leftsonar.ping_cm();
  if(SideSensorDistance >= 60){ //left turn check 
      LeftTurn_Reached++;
  }else if (SideSensorDistance <= 20 ){
      RightTurn_Reached++; 
  }

  if(LeftTurn_Reached > RightTurn_Reached && LeftTurn_Reached >= SAMPLESIZE){
    return turnLeft;
  }else if(RightTurn_Reached > LeftTurn_Reached && RightTurn_Reached >= SAMPLESIZE){
    return turnRight;
  }else{
    return turn;
  }

  
}

