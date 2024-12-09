#include <Wire.h>
// Adressen
const int I2C_adress_MPU = 0x68; // I2C address of MPU6050.
const int I2C_address_QMC = 0x0D; // I2C address of QMC5883

constexpr float dT = 1.0/75;    //Step size for model in kalman filter

constexpr float K[2]={ 0.1683, 0.8053 };  //Kalman Gain

// These are the offsets and gains from the magnetometer. This is needed to correct the hard-/softironing effects
const static int magnetometerMeans[3] = {7392, 1246, -4250};
const static float magnetometerGains[3] = {75.0, 371.0, 154.0};

const float xyPlaneGain = 1.0*magnetometerGains[0]/magnetometerGains[1];  //factor for length compensation on the y axis inside the atan2()
float angle = 0;  // measured angle
float vangle = 0; // measured angular velocity
const float pi = 3.14159;
const static float conversionGainIMU = 2*pi/(131*360);  //conversion factor for conversion from LSB/(°/s) to rad/s

// Variables from the kalman filter for the estimation and the correction part
float phaseAngleEst = 0;
float angularSpeedEst = 0;
float phaseAngleCorr = 0;
float angularSpeedCorr = 0;

// Raw sensor data holders
int16_t gyro_x;
int16_t mag_x, mag_y, mag_z;

// Interrupt pin and variables used in ISR
const int dataReadyPin = 3;
volatile bool newData = false;

// variable for angular velocity calibration. It is assumed that the chip isn't moved while calibration
long sum = 0;

enum QMC5883_Registers {  //All registers, see datasheet
  Data_X_LSB = 0x00,
  Data_X_MSB = 0x01,
  Data_Y_LSB = 0x02,
  Data_Y_MSB = 0x03,
  Data_Z_LSB = 0x04,
  Data_Z_MSB = 0x05,
  Data_SREG = 0x06,
  Temperature_LSB = 0x07,
  Temperature_MSB = 0x08,
  Control_Register = 0x09,
  Control_2_Register = 0x0A,
  Period_Register = 0x0B,
  SREG_2 = 0x0C,
  Chip_ID = 0x0D
};

enum MPU6050_Registers {  //Most important registers, see register map
  CONFIG = 0x1A,
  SMPLRT_DIV = 0x19,
  GYRO_CONFIG = 0x1B,
  INT_PIN_CFG = 0x37,
  INT_ENABLE = 0x38,
  INT_STATUS = 0x03A,
  ACCEL_XOUT_H = 0x3B,
  ACCEL_XOUT_L = 0x3C,
  ACCEL_YOUT_H = 0x3D,
  ACCEL_YOUT_L = 0x3E,
  ACCEL_ZOUT_H = 0x3F,
  ACCEL_ZOUT_L = 0x40,
  TEMP_OUT_H = 0x41,
  TEMP_OUT_L = 0x42,
  GYRO_XOUT_H = 0x43,
  GYRO_XOUT_L = 0x44,
  GYRO_YOUT_H = 0x45,
  GYRO_YOUT_L = 0x46,
  GYRO_ZOUT_H = 0x47,
  GYRO_ZOUT_L = 0x48,
  SIGNAL_PATH_RESET = 0x68,
  PWR_MGMNT_1 = 0x6B,
  WHO_AM_I = 0x75
};



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Wire.begin();
  Wire.beginTransmission(I2C_address_QMC);
  Wire.write(Control_Register);           // Put magnetometer in run mode
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.beginTransmission(I2C_adress_MPU); 
  Wire.write(GYRO_CONFIG);
  Wire.write(0x00);                     // Gyroscope 250 deg/s mode
  Wire.endTransmission(true);
  Wire.beginTransmission(I2C_adress_MPU);
  Wire.write(PWR_MGMNT_1);
  Wire.write(0x80);                       // Reset
  Wire.endTransmission(true);
  Wire.beginTransmission(I2C_adress_MPU); 
  Wire.write(SIGNAL_PATH_RESET);
  Wire.write(0x07);                       // Reset registers
  Wire.endTransmission(true);
  delay(20);
  Wire.beginTransmission(I2C_adress_MPU); 
  Wire.write(PWR_MGMNT_1);              // Reset Chip
  Wire.write(0x00);
  Wire.endTransmission(true);
  Wire.beginTransmission(I2C_adress_MPU); 
  Wire.write(CONFIG);                   // Set Bandwith to 44Hz and set frame synchronisation to AccelX
  Wire.write(0x2B);
  Wire.endTransmission(true);
  Wire.beginTransmission(I2C_adress_MPU); 
  Wire.write(SMPLRT_DIV);     // Set output Rate Divider to 13 = 0x0C+1 to approximate 75Hz output frequency
  Wire.write(0x0C);
  Wire.endTransmission(true);
  Wire.beginTransmission(I2C_adress_MPU); // enables hardware pin interrupt
  Wire.write(INT_PIN_CFG);
  Wire.write(0x14);
  Wire.endTransmission(true);
  Wire.beginTransmission(I2C_adress_MPU); 
  Wire.write(INT_ENABLE);
  Wire.write(0x01);                     // Enable interrupt for data ready pin
  Wire.endTransmission(true);

  // set up interrupt
  pinMode(dataReadyPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(dataReadyPin), readData, RISING);

  // perform calibration, offset error only
  for(int i=0; i<300;++i) {
    Wire.beginTransmission(I2C_adress_MPU);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_adress_MPU, 2, true);
    gyro_x = Wire.read()<<8 | Wire.read();
    if (i > 90) {
      sum += gyro_x;
    }
  }
  sum/=210;
}


void loop() {
  //only calculate when new data is available
  if (newData) {
    // read in the values
    Wire.beginTransmission(I2C_adress_MPU);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_adress_MPU, 2, true);
    gyro_x = Wire.read()<<8 | Wire.read();
    Wire.beginTransmission(I2C_address_QMC);
    Wire.write(Data_X_LSB);
    Wire.endTransmission();
    Wire.requestFrom(I2C_address_QMC, 6, true);
    if(Wire.available() <= 6) { // strangely enough, all values need to be read in order to make the chip update the values, maybe a chip error
      mag_x = Wire.read() | Wire.read()<<8;
      mag_y = Wire.read() | Wire.read()<<8;
      mag_z = Wire.read() | Wire.read()<<8;
    }
    
    angle = atan2(xyPlaneGain*(mag_y-magnetometerMeans[1]), (mag_x-magnetometerMeans[0]));  // calculate angle on offset and gain corrected magnetometer data

    vangle = conversionGainIMU*(gyro_x-sum);  // convert angular velocity to rad/s


    // Kalman Filter
    // estimation
    phaseAngleEst = phaseAngleCorr + angularSpeedCorr*dT;
    angularSpeedEst = angularSpeedCorr;

    //correction
    phaseAngleCorr = phaseAngleEst + K[1]*(angle-phaseAngleEst);
    angularSpeedCorr = angularSpeedEst + K[1]*(vangle-angularSpeedEst);

    // output data
    Serial.print("vangle:");Serial.println(vangle);
    Serial.print("vest:");Serial.println(angularSpeedCorr);
    newData = false;
  }
}

// isr for data reading
void readData() {

  newData = true;
}
