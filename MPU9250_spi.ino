#include <SPI.h>

#include "I2Cdev.h"
uint8_t MAG_ADR = 0x0c;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define MOSI_PIN 11
#define SCK_PIN  13
#define SS_PIN   10  //cs
#define MISO_PIN 12

#define AK8963

uint8_t buffer[14];  
int16_t ax,ay,az;
int16_t p,q,r;
int16_t temp;
int16_t mx,my,mz;

void setup() {
  
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 2; // 24: 400kHz I2C clock (200kHz if CPU is 8MHz) //2014.01.10変えてみた．
    //TWBR = 12; // 12;400kHz I2C clock (400kHz if CPU is 16MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
  
  Serial.begin(115200);

  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  
  pinMode(SS_PIN, OUTPUT);
  
  uint8_t who_am_i;

  //  I2Cdev::readByte(ADR, 0x75, &who_am_i);
  digitalWrite(SS_PIN,LOW);
  uint8_t dump;
  dump = SPI.transfer(0xF5); // READ(MSB=1) 0x80 or 0x75 -> 0xF5
  who_am_i = SPI.transfer(0);
  digitalWrite(SS_PIN, HIGH);
  
  if(who_am_i == 0x71){
      Serial.println("Successfully connected to MPU9250");

      // I2Cdev::writeByte(ADR, 0x1b, 0x00);
      // GYRO_CONFIG(0x1B) -> 0x00
      // Full Scale Accelerometer Range  = 250 deg/s
      digitalWrite(SS_PIN,LOW);
      SPI.transfer(0x1b); //  Write(MSB=0)
      SPI.transfer(0x00);
      digitalWrite(SS_PIN,HIGH);
      delay(1);
      
      // I2Cdev::writeByte(ADR, 0x1c, 0x00);
      // ACCEL_CONFIG(0x1C) -> 0x00
      digitalWrite(SS_PIN,LOW);
      SPI.transfer(0x1c);
      SPI.transfer(0x00);
      digitalWrite(SS_PIN,HIGH);
      delay(1);
      
      // I2Cdev::writeByte(ADR, 0x6b, 0x00);
      // PWR_MGMT_1(0x6B) -> 0x00
      digitalWrite(SS_PIN,LOW);
      SPI.transfer(0x6b);
      SPI.transfer(0x00);
      digitalWrite(SS_PIN,HIGH);
      delay(1);

 }
  else{
      Serial.println("Failed to Connect to MPU9250");
  }

  // Connect to COMPASS(AK8963) via AUX_i2c 

#ifdef AK8963 
  I2Cdev::readByte(MAG_ADR,0x00,&who_am_i);
  if(who_am_i == 0x48){
     Serial.println("Successfully connected to COMPASS(AK8963)");
     // CONTROL(0x0A) -> 0x00
     // Reset Magnet Compass
     I2Cdev::writeByte(MAG_ADR, 0x0a, 0x00);
     delay(1);
     // CONTROL(0x0A) -> 0x16
     // BIT       = 1  -> 16 bit output
     // MODE[3:0] = 0110b -> continuous measurement mode 2(100Hz);
     I2Cdev::writeByte(MAG_ADR, 0x0a, 0x16);
     delay(11);
  }
  else{
      Serial.println("Failed to Connect to COMPASS(AK8963)");
  }
#endif

}

void loop() {

  uint8_t dump;
  uint8_t who_am_i;
  int16_t i;
  digitalWrite(SS_PIN,LOW);
//  dump = SPI.transfer(0xF5);
//  who_am_i = SPI.transfer(0);
  SPI.transfer(0xbb);
  for(i=0;i<14;i++){
    buffer[i] = SPI.transfer(0x00);
  }
  digitalWrite(SS_PIN, HIGH);

  ax = (((int16_t)buffer[0]) << 8) | buffer[1];
  ay = (((int16_t)buffer[2]) << 8) | buffer[3];
  az = (((int16_t)buffer[4]) << 8) | buffer[5];
  temp = (((int16_t)buffer[6]) << 8) | buffer[7];
  p = (((int16_t)buffer[8]) << 8) | buffer[9];
  q = (((int16_t)buffer[10]) << 8) | buffer[11];
  r = (((int16_t)buffer[12]) << 8) | buffer[13];
  
  uint8_t mag_status2;

#ifdef AK8963
  I2Cdev::readBytes(MAG_ADR, 0x03, 6, buffer);
  I2Cdev::readByte(MAG_ADR, 0x09, &mag_status2);

  mx = (((int16_t)buffer[1]) << 8) | buffer[0];
  my = (((int16_t)buffer[3]) << 8) | buffer[2];
  mz = (((int16_t)buffer[5]) << 8) | buffer[4];
#endif

    Serial.print(ax);    Serial.print(",");
    Serial.print(ay);    Serial.print(",");
    Serial.print(az);    Serial.print(",");
    Serial.print(p);    Serial.print(",");
    Serial.print(q);    Serial.print(",");
    Serial.print(r);    Serial.print(",");
    Serial.print(mx);    Serial.print(",");
    Serial.print(my);    Serial.print(",");
    Serial.print(mz);    Serial.print(",");
    Serial.println(temp);
  
  delay(100);
  
  
}
