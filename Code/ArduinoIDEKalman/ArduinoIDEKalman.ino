#include <Wire.h>
// Adressen
const int I2C_adress_MPU = 0x68; // I2C address of MPU6050.
const int I2C_address_QMC = 0x0D; // I2C address of QMC5883

constexpr float dT = 1.0/75;
constexpr float A[2][2] = {
  {1, dT},
  {0, 1}
};

constexpr float K[2][1]={
  {0.1683},
  {0.8053}
};

volatile int16_t gyro_x;
volatile int16_t mag_x, mag_y;

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

const int dataReadyPin = 3;
const int intPin = A7;
volatile bool newData = false;
//set up timer
int timer1_cmp_match = 15624; // for 4Hz approximately on a prescaler of 256
ISR(TIMER1_COMPA_vect);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting setup...");
  Wire.begin();
  Wire.beginTransmission(I2C_address_QMC);
  Wire.write(Control_Register);
  Wire.write(0x01);
  Wire.endTransmission();
  Serial.println("Finished setting up the QMC, setting up MPU...");
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
  Wire.write(PWR_MGMNT_1);
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
  Wire.write(0x74);
  Wire.endTransmission(true);
  Wire.beginTransmission(I2C_adress_MPU); 
  Wire.write(INT_ENABLE);
  Wire.write(0x01);                     // Enable interrupt for data ready pin
  Wire.endTransmission(true);
  Serial.println("Finished setting up MPU");

  // set up interrupt
  pinMode(dataReadyPin, INPUT);
  pinMode(intPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(dataReadyPin), readData, RISING);

  // noInterrupts();
  //Set up Timer
  // TCCR2B &= ~((1 << CS22) | (1 << CS21)); //sets bits 2 and 3 to zero, for the naming, see datasheet
  // TCCR1A = 0;
  // TCCR1B = 0;
  // TCNT1 = timer1_cmp_match;
  // TCCR1B |= (1 << CS12);      //set prescaler to 256 (see datasheet)
  // TIMSK1 |= (1 << OCIE1A);    //enable timer interrupt for compare mode
  // TCCR2B |= 0x01; //starts counter (sets bit 1 to 1)

  // interrupts();
  Serial.println("Finished general setup");
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t status;
  // Wire.beginTransmission(I2C_adress_MPU); 
  // Wire.write(INT_STATUS); 
  // Wire.endTransmission(false);
  // Wire.requestFrom(I2C_adress_MPU, 1, true);
  //   status = Wire.read();
  Serial.print("x: ");Serial.println(analogRead(intPin));
  if (newData) {
    Serial.println("Got new Data");
    newData = false;
  }
}

void readData() {
  Wire.beginTransmission(I2C_adress_MPU);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
    Wire.requestFrom(I2C_adress_MPU, 2, true);
    gyro_x = Wire.read()<<8 | Wire.read();

  Serial.println(gyro_x);
  newData = true;
}

ISR(TIMER1_COMPA_vect) {
  Wire.beginTransmission(I2C_adress_MPU);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
    Wire.requestFrom(I2C_adress_MPU, 2, true);
    gyro_x = Wire.read()<<8 | Wire.read();

  Serial.println(gyro_x);
  newData = true;
  TCNT1 = timer1_cmp_match; //reload timer
}
