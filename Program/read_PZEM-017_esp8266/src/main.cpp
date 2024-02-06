#include <Arduino.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h> //  ( Master1MCU ESP8266 )

//=========Pinout I/O===========
// RS485
#define MAX485_RO 13
#define MAX485_RE 12
#define MAX485_DE 14
#define MAX485_DI 16

float PZEMVoltage, PZEMCurrent, PZEMPower, PZEMEnergy, batteryPercentage, batteryLoad, batteryCapacity, duration, power;

// Address PZEM-017 : 0x01-0xF7
static uint8_t pzemSlaveAddr = 0x00;
// Code shunt -->> 0x0000-100A, 0x0001-50A, 0x0002-200A, 0x0003-300A
static uint16_t NewshuntAddr = 0x0000;

ModbusMaster Master1;

uint8_t result;
uint16_t data[6];

SoftwareSerial pzem(MAX485_RO, MAX485_DI); // D5:RO/RX  & D6:DI/TX

void preTransmission()
{
  digitalWrite(MAX485_RE, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);
}

void setShunt(uint8_t slaveAddr) // Change the slave address of a Master1
{

  /* 1- PZEM-017 DC Energy Meter */

  static uint8_t SlaveParameter = 0x06;     /* Write command code to PZEM */
  static uint16_t registerAddress = 0x0003; /* change shunt register address command code */

  uint16_t u16CRC = 0xFFFF;                 /* declare CRC check 16 bits*/
  u16CRC = crc16_update(u16CRC, slaveAddr); // Calculate the crc16 over the 6bytes to be send
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewshuntAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewshuntAddr));

  preTransmission(); /* trigger transmission mode*/

  pzem.write(slaveAddr); /* these whole process code sequence refer to manual*/
  pzem.write(SlaveParameter);
  pzem.write(highByte(registerAddress));
  pzem.write(lowByte(registerAddress));
  pzem.write(highByte(NewshuntAddr));
  pzem.write(lowByte(NewshuntAddr));
  pzem.write(lowByte(u16CRC));
  pzem.write(highByte(u16CRC));
  delay(10);
  postTransmission(); /* trigger reception mode*/
  delay(100);
}

void resetEnergy() // reset energy for Meter 1
{
  uint16_t u16CRC = 0xFFFF;           /* declare CRC check 16 bits*/
  static uint8_t resetCommand = 0x42; /* reset command code*/
  uint8_t slaveAddr = pzemSlaveAddr;  // if you set different address, make sure this slaveAddr must change also
  u16CRC = crc16_update(u16CRC, slaveAddr);
  u16CRC = crc16_update(u16CRC, resetCommand);
  preTransmission();            /* trigger transmission mode*/
  pzem.write(slaveAddr);        /* send device address in 8 bit*/
  pzem.write(resetCommand);     /* send reset command */
  pzem.write(lowByte(u16CRC));  /* send CRC check code low byte  (1st part) */
  pzem.write(highByte(u16CRC)); /* send CRC check code high byte (2nd part) */
  delay(10);
  postTransmission(); /* trigger reception mode*/
  delay(100);
}

void pzemdata()
{

  // Master1.clearResponseBuffer();
  result = Master1.readInputRegisters(0x0000, 6);
  if (result == Master1.ku8MBSuccess)
  {
    uint32_t tempdouble = 0x00000000;
    PZEMVoltage = Master1.getResponseBuffer(0x0000) / 100.0;
    PZEMCurrent = Master1.getResponseBuffer(0x0001) / 100.0;

    tempdouble = (Master1.getResponseBuffer(0x0003) << 16) + Master1.getResponseBuffer(0x0002);
    PZEMPower = tempdouble / 10.0;

    tempdouble = (Master1.getResponseBuffer(0x0005) << 16) + Master1.getResponseBuffer(0x0004);
    PZEMEnergy = tempdouble;

    Serial.print("VOLTAGE:           ");
    Serial.println(PZEMVoltage); // V
    Serial.print("CURRENT_USAGE:     ");
    Serial.println(PZEMCurrent, 3); //  A
    Serial.print("ACTIVE_POWER:      ");
    Serial.println(PZEMPower); //  W
    Serial.print("ACTIVE_ENERGY:     ");
    Serial.println(PZEMEnergy, 3); // kWh
    Serial.println("====================================================");

    delay(1000);
  }
  else
  {
    Serial.println("Komunikasi gagal!,Periksa terminasi serial");
  }
}

void setup()
{

  pinMode(MAX485_RE, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  // Init in receive mode
  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);

  Serial.begin(115200);
  pzem.begin(9600);

  // Modbus slave ID 1
  Master1.begin(pzemSlaveAddr, pzem);

  // Callbacks allow us to configure the RS485 transceiver correctly
  Master1.preTransmission(preTransmission);
  Master1.postTransmission(postTransmission);
}

void loop()
{
  pzemdata();
}