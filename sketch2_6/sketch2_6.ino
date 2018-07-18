#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
#define LED_PIN 13 
#define factor_conversion 0.0012
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// offset
int counter_calibracion = 0;
int ax_off, ay_off, az_off;
int gx_off, gy_off, gz_off;

// orientation/motion vars
Quaternion quat;        // [w, x, y, z]         quaternion container        
int gx, gy, gz;         // [x, y, z]            raw gyroscope measurements
VectorInt16 accel;      // [x, y, z]            raw accelerometer measurements  
VectorInt16 accelReal;  // [x, y, z]            gravity-free accel sensor measurements        
VectorInt16 accelWorld; // [x, y, z]            world linear accel sensor measurements
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
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    mpu.initialize();
        
    // wait for ready. It will start once a character is sent
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again


    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {

    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

void calibrar_accel() {

  ax_off = mpu.getXAccelOffset();
  ay_off = mpu.getYAccelOffset();
  az_off = mpu.getZAccelOffset();

  if (accel.x > 0) ax_off--; else ax_off++;
  if (accel.y > 0) ay_off--; else ay_off++;
  if (accel.z > 0) az_off--; else az_off++;
    
  mpu.setXAccelOffset(ax_off);
  mpu.setYAccelOffset(ay_off);
  mpu.setZAccelOffset(az_off);
  
}


//--------------READINGS------------------//


 String readSensor() {

  int16_t aReal_current_x_sum;
  int16_t aReal_current_y_sum;
  int16_t aReal_current_z_sum;

  int16_t aWorld_current_x_sum;
  int16_t aWorld_current_y_sum;
  int16_t aWorld_current_z_sum;

  int i;
  for (i=0; i < 9; i++) {
    
    mpu.dmpGetQuaternion(&quat, fifoBuffer);
    mpu.getRotation(&gx, &gy, &gz);
    mpu.dmpGetAccel(&accelReal, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &quat);
    mpu.dmpGetLinearAccel(&accelReal, &accel, &gravity); //8192 = 1g.
    mpu.dmpGetLinearAccelInWorld(&accelWorld, &accelReal, &quat);

    aReal_current_x_sum = aReal_current_x_sum + accelReal.x;
    aReal_current_y_sum = aReal_current_y_sum + accelReal.y;
    aReal_current_z_sum = aReal_current_z_sum + accelReal.z;
  
    aWorld_current_x_sum = aWorld_current_x_sum + accelWorld.x;
    aWorld_current_y_sum = aWorld_current_y_sum + accelWorld.y;
    aWorld_current_z_sum = aWorld_current_z_sum + accelWorld.z;

    delay(1);

  }

  accelReal.x = aReal_current_x_sum/9;
  accelReal.y = aReal_current_y_sum/9;
  accelReal.z = aReal_current_z_sum/9;

  accelWorld.x = aWorld_current_x_sum/9;
  accelWorld.y = aWorld_current_y_sum/9;
  accelWorld.z = aWorld_current_z_sum/9;

    String accelReading = formatAccel();
    String quaternionReading = formatQuaternion();
    return String(accelReading + ";" + quaternionReading + "\n");
            
}

    String formatQuaternion() {

            String qw = String(quat.w);
            String qx = String(quat.x);
            String qy = String(quat.y);
            String qz = String(quat.z);
            return String(qw + "," + qx + "," + qy + "," + qz);
            
    }

    String formatAccel() {
      
            // display real acceleration, adjusted to remove gravity, in m/s
            String accx = String((accelReal.x - accelWorld.x)*factor_conversion); 
            String accy = String((accelReal.y - accelWorld.y)*factor_conversion);
            String accz = String((accelReal.z - accelWorld.z)*factor_conversion);
            return String(accx + "," + accy + "," + accz); 
      
    }



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
      if (!dmpReady) return;
      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();
  
      // get current FIFO count
      fifoCount = mpu.getFIFOCount();
  
      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
          // reset so we can continue cleanly
          mpu.resetFIFO();
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else if (mpuIntStatus & 0x02) {
          // wait for correct available data length, should be a VERY short wait
          while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
          // read a packet from FIFO
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          
          // track FIFO count here in case there is > 1 packet available
          // (this lets us immediately read more without waiting for an interrupt)
          fifoCount -= packetSize;
      }

    if (counter_calibracion == 100) {
      calibrar_accel();
      counter_calibracion = 0;
    }

    String formattedReading = readSensor();
    Serial.print(formattedReading);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    delay(1);
    mpu.resetFIFO();
    counter_calibracion++;
    
}
