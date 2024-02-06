#include <Arduino.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h> //  ( NODEMCU ESP8266 )

//=========Pinout I/O===========
// RS485
#define MAX485_RO 13
#define MAX485_RE 12
#define MAX485_DE 14
#define MAX485_DI 16

// Address PZEM-016 : 0x01-0xF7
static uint8_t pzemSlaveAddr = 0x02;
ModbusMaster Master1;

double Tegangan, Arus, Daya_Aktif, Energy_Aktif, Frekuensi, Faktor_Daya;
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

void pzemdata()
{

  // node.clearResponseBuffer();
  result = Master1.readInputRegisters(0x0000, 10);
  if (result == Master1.ku8MBSuccess)
  {
    Tegangan = (Master1.getResponseBuffer(0x00) / 10.0f);
    Arus = (Master1.getResponseBuffer(0x01) / 1000.000f);
    Daya_Aktif = (Master1.getResponseBuffer(0x03) / 10.0f);
    Energy_Aktif = (Master1.getResponseBuffer(0x05) / 1000.0f);
    Frekuensi = (Master1.getResponseBuffer(0x07) / 10.0f);
    Faktor_Daya = (Master1.getResponseBuffer(0x08) / 100.0f);

    Serial.print("VOLTAGE:           ");
    Serial.println(Tegangan); // V
    Serial.print("CURRENT_USAGE:     ");
    Serial.println(Arus, 3); //  A
    Serial.print("ACTIVE_POWER:      ");
    Serial.println(Daya_Aktif); //  W
    Serial.print("ACTIVE_ENERGY:     ");
    Serial.println(Energy_Aktif, 3); // kWh
    Serial.print("FREQUENCY:         ");
    Serial.println(Frekuensi); // Hz
    Serial.print("POWER_FACTOR:      ");
    Serial.println(Faktor_Daya);
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