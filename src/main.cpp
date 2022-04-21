
/*********************************************************************
 * Works great.  Sample output is below.
 * 
 * 
 * Checking for I2C devices...:
 * DS2482-100 present
 *         Checking for 1-Wire devices...
 *         Devices present on 1-Wire bus
 *         Searching 1-Wire bus...
 *         Found device: { 0x1D, 0xB8, 0x87, 0x01, 0x00, 0x00, 0x00, 0x70 }
 *         Calculated CRC of device serial is 70
 *         This device is a 1-wire counter
 * Storing this ROM ID
 * Returned channel selected 1
 *         Counter returned: 23
 * 
 *  
 */

#include <Arduino.h>
#include <Wire.h>
#include "DS248X.h"

#define CONVERT_TEMPERATURE_COMMAND       0x44
#define READ_SCRATCHPAD_COMMAND           0xBE
#define WRITE_SCRATCHPAD_COMMAND          0x4e
#define TEMPERATURE_CONVERSION_DELAY      750
#define DELAY_ADD                         0

DS248X owMaster;
void printAddress(uint8_t[]);
void printArrayInHex(uint8_t[], int len);
uint8_t getCRC8( uint8_t *addr, uint8_t len);
uint16_t getCRC16( uint8_t *data, uint8_t len);
void readCounter(uint8_t *addr);
bool readTemperatureDevice(uint8_t *addr);

uint8_t currentAddress[8];

char buffer[256];
char temp_str[80];

void setup()
{
  Serial.begin(9600);
  
  owMaster.DS2482Reset();
  owMaster.writeConfig(0x01);
  owMaster.selectChannel(0);   

  delay(2000);
}

void loop()
{        
  uint8_t crc8;

  Serial.println("\nChecking for I2C devices...:");
  if (owMaster.checkPresence())
  {
    Serial.println("DS2482-100/800 present");  
       
    Serial.println("\tChecking for 1-Wire devices...");
    if (owMaster.OWReset())
    {
      Serial.println("\tDevices present on 1-Wire bus");            
      Serial.println("\tSearching 1-Wire bus...");      
      while (owMaster.OWSearch(currentAddress)) {
        Serial.print("\tFound device: ");
        printAddress(currentAddress);
        Serial.println();
        // Calculate the CRC (if the CRC byte, the 8th, byte is included
	      // in the CRC calculation then getCRC8() will return 0)
	      crc8 = getCRC8(currentAddress, 7);  
	      Serial.print("\tCalculated CRC of device serial is ");
	      Serial.println(crc8, HEX);

        if (currentAddress[0] == 0x1D) {
          Serial.print("\tFound 1-wire counter at ");
          printAddress(currentAddress);
          Serial.println("");                    
          readCounter(currentAddress);                          
        } 
        if (currentAddress[0] == 0x10 || currentAddress[0] == 0x26) {
          Serial.println("\tThis device is a 1-wire thermometer");
          if (!readTemperatureDevice(currentAddress)) {
            Serial.println("temperature reading error");
          }            
        }          
      }      
      owMaster.OWResetSearch();      
    }
    else
      Serial.println("\tNo devices on 1-Wire bus");
  }
  else
    Serial.println("No DS2482 present");

  delay(5000);
}


void readCounter(uint8_t *addr) {
    uint32_t count = 0;

    // Internal buffer
    uint8_t uint8_buffer[45];
    
    /**
    * DS2423 read commands
    **/
    uint8_t READ_MEMORY_AND_COUNTER_COMMAND = 0xA5;
    
    int channel = owMaster.selectChannel(0);
    owMaster.OWReset();
    owMaster.OWSelect(addr);
    uint8_buffer [0] = READ_MEMORY_AND_COUNTER_COMMAND;
    uint8_buffer [1] = 0xC0;    
    uint8_buffer [2] = 0x01;
    
    owMaster.OWWriteByte(uint8_buffer[0], 0);
    owMaster.OWWriteByte(uint8_buffer[1], 0);
    owMaster.OWWriteByte(uint8_buffer[2], 0);

    for (uint8_t i = 3; i < 45; i++)
        uint8_buffer [i] = owMaster.OWReadByte();

    owMaster.OWReset();

    count = (uint32_t)uint8_buffer[38];
    for (int j = 37; j >= 35; j--) {
        count = (count << 8) + (uint32_t)uint8_buffer[j];
    }

    uint16_t crc = getCRC16(uint8_buffer, 43);
    uint8_t *crcBytes = (uint8_t *)&crc;
    uint8_t crcLo = ~uint8_buffer[43];
    uint8_t crcHi = ~uint8_buffer[44];
    
    int error = 0;
    error = (crcLo != crcBytes[0]) || (crcHi != crcBytes[1]);
    Serial.print("\tDS2423 returned: ");
    Serial.println(count);     
}


bool readTemperatureDevice(uint8_t *addr) {   
  double tempInC;
  double tempInF;

  uint8_t data[9];
  uint8_t CRCerror = 0;
  
  if (addr[0] == 0x10) { 
    if (owMaster.OWReset()) {           //Reset was successful
      owMaster.selectChannel(0);
      owMaster.OWSelect(addr);
      owMaster.setStrongPullup();
      owMaster.OWWriteByte(CONVERT_TEMPERATURE_COMMAND);          
      delay(TEMPERATURE_CONVERSION_DELAY + DELAY_ADD); 
      owMaster.clearStrongPullup();

      owMaster.OWReset();
      owMaster.OWSelect(addr);      
      owMaster.OWWriteByte(READ_SCRATCHPAD_COMMAND);  //Read Scratchpad   

      // Getting temperature info from device state...
      // we need 9 uint8_t's
      for (int i = 0; i < 9; i++) {         
        data[i] = owMaster.OWReadByte();
      }

      uint8_t lsb = data[0];
      uint8_t msb = data[1];
      int16_t temp = (msb << 8) + lsb;
      uint8_t sign = temp & 0x8000;
      if (sign) {
          temp = (temp ^ 0xffff) + 1;
      }

      tempInC = (double)temp / 2.0 + 0.05;
      if (sign) {
         tempInC = 0.0 - tempInC;
      }

      CRCerror = data[8] != getCRC8(data, 8);

      if (CRCerror) {
        Serial.println("\tCRC error reading temperature");
        return false;
      }

      if (tempInC > 85) {
        Serial.println("\tTemperature read error: over-range! ");
        return false;
      }
      else {
        Serial.print("\tNo temperature error. ");
        Serial.print("Temperature = ");
        Serial.print(tempInC, 1);
        Serial.print("C or ");
        tempInF = (9 * tempInC / 5) + 32;
        Serial.print(tempInF, 1);
        Serial.println("F");
      }    
    }
  } else 
    if (addr[0] == 0x26) {
      Serial.println("\tThis device is a DS2438 ADC/thermometer");
      Serial.println("\tNo code here yet"); 
    }  
  return true;
}


void printAddress(uint8_t *deviceAddress)
{
  Serial.print("{ ");
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    Serial.print("0x");
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i<7) Serial.print(", ");
  }
  Serial.print(" }");
}


void printArrayInHex (uint8_t array[], int len)
{
  Serial.print("{ ");
  for (uint8_t i = 0; i < len; i++)
  {
    // zero pad the address if necessary
    Serial.print("0x");
    if (array[i] < 16) Serial.print("0");
    Serial.print(array[i], HEX);
    if ( !(i % 8) ) Serial.print("");
    if (i < len - 1) Serial.print(", ");    
  }
  Serial.print(" }");
}

