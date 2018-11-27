#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu(0x68);
MPU6050 mpu_1(0x68);
MPU6050 mpu_2(0x68);
MPU6050 mpu_3(0x68);

#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
int s0 = 5;
int s1 = 6;

// ================= MPU: 0 =================
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// ================= MPU: 1 =================
// MPU control/status vars
bool dmpReady_1 = false;  // set true if DMP init was successful
uint8_t mpuIntStatus_1;   // holds actual interrupt status byte from MPU
uint8_t devStatus_1;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize_1;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount_1;     // count of all bytes currently in FIFO
uint8_t fifoBuffer_1[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q_1;           // [w, x, y, z]         quaternion container
VectorFloat gravity_1;    // [x, y, z]            gravity vector
float ypr_1[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt_1 = false;     // indicates whether MPU interrupt pin has gone high

// ================= MPU: 2 =================
// MPU control/status vars
bool dmpReady_2 = false;  // set true if DMP init was successful
uint8_t mpuIntStatus_2;   // holds actual interrupt status byte from MPU
uint8_t devStatus_2;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize_2;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount_2;     // count of all bytes currently in FIFO
uint8_t fifoBuffer_2[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q_2;           // [w, x, y, z]         quaternion container
VectorFloat gravity_2;    // [x, y, z]            gravity vector
float ypr_2[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt_2 = false;     // indicates whether MPU interrupt pin has gone high

// ================= MPU: 3 =================
// MPU control/status vars
bool dmpReady_3 = false;  // set true if DMP init was successful
uint8_t mpuIntStatus_3;   // holds actual interrupt status byte from MPU
uint8_t devStatus_3;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize_3;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount_3;     // count of all bytes currently in FIFO
uint8_t fifoBuffer_3[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q_3;           // [w, x, y, z]         quaternion container
VectorFloat gravity_3;    // [x, y, z]            gravity vector
float ypr_3[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt_3 = false;     // indicates whether MPU interrupt pin has gone high

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    pinMode(s0, OUTPUT); 
    pinMode(s1, OUTPUT); 
  
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
    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    // ================= MPU: 0 =================
    digitalWrite(s0, LOW);
    digitalWrite(s1, LOW);
    delay(20);
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
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
    delay(20);

    // ================= MPU: 1 =================
    digitalWrite(s0, HIGH);
    digitalWrite(s1, LOW);
    delay(20);
    Serial.println(F("Initializing I2C devices..."));
    mpu_1.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu_1.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus_1 = mpu_1.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu_1.setXGyroOffset(220);
    mpu_1.setYGyroOffset(76);
    mpu_1.setZGyroOffset(-85);
    mpu_1.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus_1 == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu_1.setDMPEnabled(true);

        // enable Arduino interrupt detection
        mpuIntStatus_1 = mpu_1.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady_1 = true;

        // get expected DMP packet size for later comparison
        packetSize_1 = mpu_1.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus_1);
        Serial.println(F(")"));
    }
    delay(20);

    // ================= MPU: 2 =================
    digitalWrite(s0, LOW);
    digitalWrite(s1, HIGH);
    delay(20);
    Serial.println(F("Initializing I2C devices..."));
    mpu_2.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu_2.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus_2 = mpu_2.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu_2.setXGyroOffset(220);
    mpu_2.setYGyroOffset(76);
    mpu_2.setZGyroOffset(-85);
    mpu_2.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus_2 == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu_2.setDMPEnabled(true);

        // enable Arduino interrupt detection
        mpuIntStatus_2 = mpu_2.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady_2 = true;

        // get expected DMP packet size for later comparison
        packetSize_2 = mpu_2.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus_2);
        Serial.println(F(")"));
    }
    delay(20);

    // ================= MPU: 3 =================
     digitalWrite(s0, HIGH);
    digitalWrite(s1, HIGH);
    delay(20);
    Serial.println(F("Initializing I2C devices..."));
    mpu_3.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu_3.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus_3 = mpu_3.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu_3.setXGyroOffset(220);
    mpu_3.setYGyroOffset(76);
    mpu_3.setZGyroOffset(-85);
    mpu_3.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus_3 == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu_3.setDMPEnabled(true);

        // enable Arduino interrupt detection
        mpuIntStatus_3 = mpu_3.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady_3 = true;

        // get expected DMP packet size for later comparison
        packetSize_3 = mpu_3.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus_3);
        Serial.println(F(")"));
    }
    delay(20);

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // ================= MPU: 0 =================
    digitalWrite(s0, LOW);
    digitalWrite(s1, LOW);
    delay(20);
  
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        // Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("#0#");
            Serial.print(ypr[0] * 180/M_PI); Serial.print("#");
            Serial.print(ypr[1] * 180/M_PI); Serial.print("#");
            Serial.println(ypr[2] * 180/M_PI);
        #endif
    }
    delay(20);

    // ================= MPU: 1 =================
    digitalWrite(s0, HIGH);
    digitalWrite(s1, LOW);
    delay(20);
  
    // if programming failed, don't try to do anything
    if (!dmpReady_1) return;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt_1 = false;
    mpuIntStatus_1 = mpu_1.getIntStatus();

    // get current FIFO count
    fifoCount_1 = mpu_1.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus_1 & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount_1 >= 1024) {
        // reset so we can continue cleanly
        mpu_1.resetFIFO();
        fifoCount_1 = mpu_1.getFIFOCount();
        // Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus_1 & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount_1 < packetSize_1) fifoCount_1 = mpu_1.getFIFOCount();

        // read a packet from FIFO
        mpu_1.getFIFOBytes(fifoBuffer_1, packetSize_1);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount_1 -= packetSize_1;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu_1.dmpGetQuaternion(&q_1, fifoBuffer_1);
            mpu_1.dmpGetGravity(&gravity_1, &q_1);
            mpu_1.dmpGetYawPitchRoll(ypr_1, &q_1, &gravity_1);
            Serial.print("#1#");
            Serial.print(ypr_1[0] * 180/M_PI); Serial.print("#");
            Serial.print(ypr_1[1] * 180/M_PI); Serial.print("#");
            Serial.println(ypr_1[2] * 180/M_PI);
        #endif
    }
    delay(20);

    // ================= MPU: 2 =================
    digitalWrite(s0, LOW);
    digitalWrite(s1, HIGH);
    delay(20);
  
    // if programming failed, don't try to do anything
    if (!dmpReady_2) return;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt_2 = false;
    mpuIntStatus_2 = mpu_2.getIntStatus();

    // get current FIFO count
    fifoCount_2 = mpu_2.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus_2 & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount_2 >= 1024) {
        // reset so we can continue cleanly
        mpu_2.resetFIFO();
        fifoCount_2 = mpu_2.getFIFOCount();
        // Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus_2 & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount_2 < packetSize_2) fifoCount_2 = mpu_2.getFIFOCount();

        // read a packet from FIFO
        mpu_2.getFIFOBytes(fifoBuffer_2, packetSize_2);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount_2 -= packetSize_2;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu_2.dmpGetQuaternion(&q_2, fifoBuffer_2);
            mpu_2.dmpGetGravity(&gravity_2, &q_2);
            mpu_2.dmpGetYawPitchRoll(ypr_2, &q_2, &gravity_2);
            Serial.print("#2#");
            Serial.print(ypr_2[0] * 180/M_PI); Serial.print("#");
            Serial.print(ypr_2[1] * 180/M_PI); Serial.print("#");
            Serial.println(ypr_2[2] * 180/M_PI);
        #endif
    }
    delay(20);

    // ================= MPU: 3 =================
    digitalWrite(s0, HIGH);
    digitalWrite(s1, HIGH);
    delay(20);
  
    // if programming failed, don't try to do anything
    if (!dmpReady_3) return;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt_3 = false;
    mpuIntStatus_3 = mpu_3.getIntStatus();

    // get current FIFO count
    fifoCount_3 = mpu_3.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus_3 & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount_3 >= 1024) {
        // reset so we can continue cleanly
        mpu_3.resetFIFO();
        fifoCount_3 = mpu_3.getFIFOCount();
        // Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus_3 & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount_3 < packetSize_3) fifoCount_3 = mpu_3.getFIFOCount();

        // read a packet from FIFO
        mpu_3.getFIFOBytes(fifoBuffer_3, packetSize_3);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount_3 -= packetSize_3;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu_3.dmpGetQuaternion(&q_3, fifoBuffer_3);
            mpu_3.dmpGetGravity(&gravity_3, &q_3);
            mpu_3.dmpGetYawPitchRoll(ypr_3, &q_3, &gravity_3);
            Serial.print("#3#");
            Serial.print(ypr_3[0] * 180/M_PI); Serial.print("#");
            Serial.print(ypr_3[1] * 180/M_PI); Serial.print("#");
            Serial.println(ypr_3[2] * 180/M_PI);
        #endif
    }
    delay(20);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
