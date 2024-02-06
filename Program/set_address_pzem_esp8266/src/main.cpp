#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ModbusMaster.h>

//=========Pinout I/O===========
// RS485
#define MAX485_RO 13
#define MAX485_RE 12
#define MAX485_DE 14
#define MAX485_DI 16

static uint8_t address = 0x02;

SoftwareSerial PZEMSerial;
ModbusMaster node;
unsigned long startMillis1;
unsigned long startMillis2;

void preTransmission()
{
  if (millis() - startMillis1 > 5000) // Wait for 5 seconds as ESP Serial cause start up code crash
  {
    digitalWrite(MAX485_RE, 1);
    digitalWrite(MAX485_DE, 1);
    delay(1);
  }
}

void postTransmission()
{
  if (millis() - startMillis1 > 5000) // Wait for 5 seconds as ESP Serial cause start up code crash
  {
    delay(3);
    digitalWrite(MAX485_RE, 0);
    digitalWrite(MAX485_DE, 0);
  }
}

void changeAddress(uint8_t OldslaveAddr, uint8_t NewslaveAddr) // Change the slave address of a node
{
  static uint8_t SlaveParameter = 0x06;        /* Write command code to PZEM */
  static uint16_t registerAddress = 0x0002;    /* Modbus RTU device address command code */
  uint16_t u16CRC = 0xFFFF;                    /* declare CRC check 16 bits*/
  u16CRC = crc16_update(u16CRC, OldslaveAddr); // Calculate the crc16 over the 6bytes to be send
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewslaveAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewslaveAddr));

  preTransmission(); /* trigger transmission mode*/
  delay(10);
  PZEMSerial.write(OldslaveAddr); /* these whole process code sequence refer to manual*/
  PZEMSerial.write(SlaveParameter);
  PZEMSerial.write(highByte(registerAddress));
  PZEMSerial.write(lowByte(registerAddress));
  PZEMSerial.write(highByte(NewslaveAddr));
  PZEMSerial.write(lowByte(NewslaveAddr));
  PZEMSerial.write(lowByte(u16CRC));
  PZEMSerial.write(highByte(u16CRC));
  delay(10);
  postTransmission(); /* trigger reception mode*/
  delay(100);
  while (PZEMSerial.available()) /* while receiving signal from Serial3 from meter and converter */
  {
  }
  Serial.println("Address Changed Successful...");
}

void setup()
{
  startMillis1 = millis();
  startMillis2 = millis();
  Serial.begin(115200);
  PZEMSerial.begin(9600, SWSERIAL_8N2, MAX485_RO, MAX485_DI);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop()
{
  if (millis() - startMillis2 > 10000) // Every 10 second set the address of the meter
  {
    changeAddress(0XF8, address); // Activate change the address
    startMillis2 = millis();
  }
}
