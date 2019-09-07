/* LimiTTer sketch for the Arduino UNO/Pro-Mini.
   It scans the Freestyle Libre Sensor every 5 minutes
   and sends the data to the xDrip Android app. You can
   see the data in the serial monitor of Arduino IDE, too.
   If you want another scan interval, simply change the
   SLEEP_TIME value. 
     
   This sketch is based on a sample sketch for the BM019 module
   from Solutions Cubed.

   Wiring for UNO / Pro-Mini:

   Arduino          ESP32  WemosD1          BM019           BLE-HM11
   IRQ: Pin 9       IO26     IO16 (D0)    DIN: pin 2
   SS: pin 10       IO5      IO15 (D8)    SS: pin 3
   MOSI: pin 11     IO23     IO13 (D7)    MOSI: pin 5 
   MISO: pin 12     IO19     IO12 (D6)    MISO: pin4
   SCK: pin 13      IO18     IO14 (D5)    SCK: pin 6
               IO16,17,21,22              VIN : pin 9
                                          GND : Pin 10
   I/O: pin 2                                  BLE_CHK: pin 15 
   I/O: pin 3                                  VCC: pin 9 
   I/O: pin 5                                  TX: pin 2
   I/O: pin 6                                  RX: pin 4
*/

#define ESP32
//#define ESP8266

#define BUG_SPI
#define PRINTMEM

#include <SPI.h>
//#include <SoftwareSerial.h>
#ifdef ESP32
//#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SERVICE_UART_UUID      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#endif


#define MAX_BLE_WAIT 90 // Maximum bluetooth re-connect time in seconds 
#define SLEEP_TIME 300000000 // SleepTime in µs 5 min = 300 s      //
#define MAX_NFC_READTRIES 10 // Amount of tries for every nfc block-scan
#define NFCTIMEOUT 500  // Timeout for NFC response
#define RXBUFSIZE 24
#define NFCMEMSIZE 320  // 40 blocs of 8 bytes
#define NFC15MINPOINTER 26  // 0x1A
#define NFC8HOURPOINTER 27  // 0x1B
#define NFC15MINADDRESS 28  // 0x1C
#define NFC8HOURADDRESS 124 // 0x64
#define NFCSENSORTIMEPOINTER  316 // 0x13C et 0x13D

byte RXBuffer[RXBUFSIZE];
byte NfcMem[NFCMEMSIZE];

byte NFCReady = 0;  // used to track NFC state
////RTC_DATA_ATTR byte FirstRun = 1;
int batteryPcnt;
long batteryMv;
int noDiffCount = 0;
int sensorMinutesElapse;
float trend[16];



#ifdef ESP8266
const int SSPin = 15;  // Slave Select pin
const int IRQPin = 16;  // Sends wake-up pulse for BM019
const int NFCPin1 = D1; // Power pin BM019
const int NFCPin2 = D2; // Power pin BM019
const int NFCPin3 = D3; // Power pin BM019
const int NFCPin4 = D4; // Power pin BM019
//const int BLEPin = 3; // BLE power pin.
//const int BLEState = 2; // BLE connection state pin
const int MOSIPin = 13;
const int SCKPin = 14;
byte FirstRun = 1;
float lastGlucose;
#endif
#ifdef ESP32
const int SSPin = 5;  // Slave Select pin
const int IRQPin = 26;  // Sends wake-up pulse for BM019
const int NFCPin1 = 16; // Power pin BM019
const int NFCPin2 = 17; // Power pin BM019
const int NFCPin3 = 21; // Power pin BM019
const int NFCPin4 = 22; // Power pin BM019
const int MOSIPin = 23;
const int SCKPin = 19;
unsigned long boottime;
unsigned long sleeptime;
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR byte FirstRun = 1;
RTC_DATA_ATTR float lastGlucose;

BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
BLECharacteristic* pRxCharacteristic = NULL;
bool estConnecte = false;
bool etaitConnecte = false;
//===================================================================================================================

class EtatServeur : public BLEServerCallbacks 
{
    void onConnect(BLEServer* pServer) 
    {
      estConnecte = true;
    }

    void onDisconnect(BLEServer* pServer) 
    {
      estConnecte = false;
    }
};

class CharacteristicUART : public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic *pCharacteristique) 
    {
      std::string rxValue = pCharacteristique->getValue();

      if (rxValue.length() > 0) 
      {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);
        Serial.println();
        Serial.println("*********");
      }
    }
};

//===================================================================================================================


#endif


void setup() {
//    boottime = millis();
    pinMode(IRQPin, OUTPUT);
    digitalWrite(IRQPin, HIGH); 
    pinMode(SSPin, OUTPUT);
    digitalWrite(SSPin, HIGH);
    pinMode(NFCPin1, OUTPUT);
    pinMode(NFCPin2, OUTPUT);
    pinMode(NFCPin3, OUTPUT);
    pinMode(NFCPin4, OUTPUT);
    digitalWrite(NFCPin1, HIGH);
    digitalWrite(NFCPin2, HIGH);
    digitalWrite(NFCPin3, HIGH);
    digitalWrite(NFCPin4, HIGH);

    pinMode(MOSIPin, OUTPUT);
    pinMode(SCKPin, OUTPUT);

    Serial.begin(9600);
#ifdef ESP32

  BLEDevice::init("LimiTTer");
  //BLEDevice::getAddress(); // Retrieve our own local BD BLEAddress
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new EtatServeur());
  
  BLEService *pServiceUART = pServer->createService(SERVICE_UART_UUID);
  pTxCharacteristic = pServiceUART->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  // Create a BLE Descriptor : Client Characteristic Configuration (for indications/notifications)
  pTxCharacteristic->addDescriptor(new BLE2902());
  pRxCharacteristic = pServiceUART->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new CharacteristicUART());
  
  pServiceUART->start();

  pServer->getAdvertising()->start();
  //BLEAdvertising *pAdvertising = pServer->getAdvertising();
  //pAdvertising->start();
  Serial.println("UART Over BLE start advertising");
  Serial.println("UART Over BLE wait connection");
    for (int i=0; ( (i < MAX_BLE_WAIT) && !estConnecte ); i++)
    {
      delay(1000);
      Serial.println("Waiting for BLE connection ...");
    }
#endif

    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV128);
    SPI.begin();

    delay(10);                      // send a wake up
    digitalWrite(IRQPin, LOW);      // pulse to put the 
    delayMicroseconds(100);         // BM019 into SPI
    digitalWrite(IRQPin, HIGH);     // mode 
    delay(10);
    digitalWrite(IRQPin, LOW);
    
     //Increment boot number and print it every reboot
////    ++bootCount;
////    Serial.println("Boot number: " + String(bootCount));
}

#ifdef BUG_SPI
void Shift_RXBuf(int LastLSb) {
  int NewMSB = LastLSb * 0x80;
  for(int i= 0; i< RXBUFSIZE;i++) {
    LastLSb = RXBuffer[i] & 0x01;
    RXBuffer[i] = (RXBuffer[i]>> 1) + NewMSB;
    NewMSB = LastLSb * 0x80;
  }
}
#endif

void poll_NFC_UntilResponsIsReady()
{
  unsigned long ms = millis();
  byte rb;
//  print_state("poll_NFC_UntilResponsIsReady() - ");
  digitalWrite(SSPin , LOW);
  while ( (RXBuffer[0] != 8) && ((millis() - ms) < NFCTIMEOUT) )
  {
#ifdef BUG_SPI  
    rb = RXBuffer[0] = SPI.transfer(0x03)>>1;  // Write 3 until
#else
    rb = RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
#endif
//    Serial.printf("SPI polling response byte:%x\r\n", RXBuffer[0]);
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
  }
  digitalWrite(SSPin, HIGH);
  delay(1);
  if ( millis() - ms > NFCTIMEOUT ) {
    Serial.print("\r\n *** poll timeout *** -> response ");
    Serial.print(rb);
  }
}

void receive_NFC_Response()
{
//  print_state("receive_NFC_Response()");
  byte datalength;
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);
  RXBuffer[0] = SPI.transfer(0);
  datalength = RXBuffer[1] = SPI.transfer(0);
#ifdef BUG_SPI  
  datalength=datalength/2;
#endif  
  for (byte i = 0; i < datalength; i++) RXBuffer[i + 2] = SPI.transfer(0);
  digitalWrite(SSPin, HIGH);
  delay(1);
#ifdef BUG_SPI  
  Shift_RXBuf(1);
#endif
}

void SetProtocol_Command() {
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x02);  // Set protocol command
  SPI.transfer(0x02);  // length of data to follow
  SPI.transfer(0x01);  // code for ISO/IEC 15693
  SPI.transfer(0x0D);  // Wait for SOF, 10% modulation, append CRC
  digitalWrite(SSPin, HIGH);
  delay(10);
 
  poll_NFC_UntilResponsIsReady();

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read         
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
  digitalWrite(SSPin, HIGH);

  if ((RXBuffer[0] == 0) & (RXBuffer[1] == 0)) {  // is response code good?
    Serial.println("Protocol Set Command OK");
    NFCReady = 1; // NFC is ready
  } else {
    Serial.println("Protocol Set Command FAIL");
    NFCReady = 0; // NFC not ready
  }
}

void Inventory_Command() {
  Serial.println("Inventory Command");
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows is 0
  SPI.transfer(0x26);  // request Flags byte
  SPI.transfer(0x01);  // Inventory Command for ISO/IEC 15693
  SPI.transfer(0x00);  // mask length for inventory command
  digitalWrite(SSPin, HIGH);
  delay(1);

  poll_NFC_UntilResponsIsReady();
  receive_NFC_Response();

  if (RXBuffer[0] == 0x80 && RXBuffer[1] == 13 )  // is response code good?
    {

#ifdef PRINTMEM  
 byte oneBlock[8];
 for (int i = 0; i < 8; i++)
   oneBlock[i] = RXBuffer[i+3];
  
  char str[24];
  unsigned char * pin = oneBlock;
  const char * hex = "0123456789ABCDEF";
  char * pout = str;
  for(; pin < oneBlock+8; pout+=2, pin++) {
      pout[0] = hex[(*pin>>4) & 0xF];
      pout[1] = hex[ *pin     & 0xF];
  }
  pout[0] = 0;
  Serial.println("Inventory : " + String(str));
 
#endif
      
    Serial.println("Sensor in range ... OK");
    NFCReady = 2;
    }
  else
    {
    Serial.println("Sensor out of range");
    NFCReady = 1;
    }
 }
 
float Read_Memory() {
Serial.println("Read Memory");
 byte oneBlock[8];
 String hexPointer = "";
 String trendValues = "";
 String hexMinutes = "";
 String elapsedMinutes = "";
 float trendOneGlucose;
 float trendTwoGlucose;
 float currentGlucose;
 float shownGlucose;
 float averageGlucose = 0;
 int glucosePointer;
 int histoPointer;
 int validTrendCounter = 0;
 int raw;
 float validTrend[16];
 byte readError = 0;
 int readTry;
 
#ifdef PRINTMEM    
 for ( int b = 3; b < 40; b++) {
#else
 for ( int b = 3; b < 16; b++) {
#endif
  readTry = 0;
  do {
    readError = 0;   
    digitalWrite(SSPin, LOW);
    SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
    SPI.transfer(0x04);  // Send Receive CR95HF command
    SPI.transfer(0x03);  // length of data that follows
    SPI.transfer(0x02);  // request Flags byte
    SPI.transfer(0x20);  // Read Single Block command for ISO/IEC 15693
    SPI.transfer(b);  // memory block address
    digitalWrite(SSPin, HIGH);
    delay(10);
   
    poll_NFC_UntilResponsIsReady();
    receive_NFC_Response();
     
   if (RXBuffer[0] != 0x80)
       readError = 1;  
    
   for (int i = 0; i < 8; i++) {
     oneBlock[i] = RXBuffer[i+3];
     NfcMem[8*b+i]=RXBuffer[i+3];
   }
    char str[24];
    unsigned char * pin = oneBlock;
    const char * hex = "0123456789ABCDEF";
    char * pout = str;
    for(; pin < oneBlock+8; pout+=2, pin++) {
        pout[0] = hex[(*pin>>4) & 0xF];
        pout[1] = hex[ *pin     & 0xF];
    }
    pout[0] = 0;
    if (!readError)       // is response code good?
    { 
#ifdef PRINTMEM  
//      Serial.println(String(str) + " Bloc " + String(b));
#endif      
      trendValues += str;
    }
    readTry++;
  } while( (readError) && (readTry < MAX_NFC_READTRIES) );
  
 }
  readTry = 0;
  do {
  readError = 0;  
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows
  SPI.transfer(0x02);  // request Flags byte
  SPI.transfer(0x20);  // Read Single Block command for ISO/IEC 15693
  SPI.transfer(39);  // memory block address
  digitalWrite(SSPin, HIGH);
  delay(10);
 
  poll_NFC_UntilResponsIsReady();

  receive_NFC_Response();

 if (RXBuffer[0] != 0x80)
     readError = 1;  
  
 for (int i = 0; i < 8; i++)
   oneBlock[i] = RXBuffer[i+3];
    
  char str[24];
  unsigned char * pin = oneBlock;
  const char * hex = "0123456789ABCDEF";
  char * pout = str;
  for(; pin < oneBlock+8; pout+=2, pin++) {
      pout[0] = hex[(*pin>>4) & 0xF];
      pout[1] = hex[ *pin     & 0xF];
  }
  pout[0] = 0;
  if (!readError) {
    Serial.println("Bloc 39 : " + String(str));
    elapsedMinutes += str;
  }
  readTry++;
  } while( (readError) && (readTry < MAX_NFC_READTRIES) );

//Serial.println("ElapsedMinutes " + elapsedMinutes);
//Serial.println("trendValues " + trendValues);
      
  if (!readError)
    {
      hexMinutes = elapsedMinutes.substring(10,12) + elapsedMinutes.substring(8,10);
      hexPointer = trendValues.substring(4,6);
//      sensorMinutesElapse = strtoul(hexMinutes.c_str(), NULL, 16);
//      glucosePointer = strtoul(hexPointer.c_str(), NULL, 16);

      sensorMinutesElapse = (NfcMem[NFCSENSORTIMEPOINTER+1]<<8) + NfcMem[NFCSENSORTIMEPOINTER];
      glucosePointer = NfcMem[NFC15MINPOINTER];
      histoPointer=NfcMem[NFC8HOURPOINTER];

Serial.println("hexMinutes : " + hexMinutes + " " + String(sensorMinutesElapse));
Serial.println("Glucose Pointer  : " + String(glucosePointer));
Serial.println("Histo Pointer  : " + String(histoPointer));

    float MeanTrend=0;
    float PenteFinale;


     for (int j=0; j<16; j++) {     
         raw = (NfcMem[NFC15MINADDRESS + 1 + ((glucosePointer+15-j)%16)*6]<<8) + NfcMem[NFC15MINADDRESS + ((glucosePointer+15-j)%16)*6];
//         trend[15-j] = Glucose_Reading((NfcMem[NFC15MINADDRESS + 1 + ((glucosePointer+j)%16)*6]<<8) + NfcMem[NFC15MINADDRESS + ((glucosePointer+j)%16)*6]) ;
        trend[j] = Glucose_Reading(raw) ;
        Serial.println("Tendance " + String((j+1)) + "minutes : " + String(trend[j]) + " Raw : " + String(raw));
        MeanTrend+=trend[j];
     }
    MeanTrend = MeanTrend/16;

    for (int j=0; j<32;j++) {
      raw = (NfcMem[NFC8HOURADDRESS + 1 + ((histoPointer+31-j)%32)*6]<<8) + NfcMem[NFC8HOURADDRESS + ((histoPointer+31-j)%32)*6];
      Serial.println("Tendance " + String((j+1)/4) + "h"+ String((j*15+15)%60) + "min : " + String(Glucose_Reading(raw)) + " Raw : " + String(raw));
    }


       
   float SigmaY = 0 ;
   for (int i = 0 ; i < 16 ; i++) { // Calcul droite avec regression des moindres carrés
       // pour les absisses, i=0 à 15 ; moyenne = 7.5 ; SigmaX = somme((i-7.5)^2) = 340
       SigmaY += (trend[i]-MeanTrend)*(i-7.5);
   }
   // Valeur lissée par la droite des moindres carrés en considérant les 15 dernière minutes comme linéaire
   // y = A x + B avec A = SigmaY/SigmaX et B =MoyenneY - A MoyenneX
   // Valeur renvoyée correspond à l'estimation pour x = 0, la valeur courante lue est remplacée par son estimation selon la droite des moindres carrés

   shownGlucose = MeanTrend - 7.5 * SigmaY/340;

    currentGlucose = trend[0];

    Serial.println("Projection moindre carrés : " + String(shownGlucose));
    Serial.println("Ecart : " + String((shownGlucose-trend[0])));
   
    if (FirstRun == 1)
       lastGlucose = trend[0];
       
    if ((lastGlucose == currentGlucose) && (sensorMinutesElapse > 21000)) // Expired sensor check
      noDiffCount++;

    if (lastGlucose != currentGlucose) // Reset the counter
      noDiffCount = 0;
    
/*    
       
    if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
    {
       if (((lastGlucose - trendOneGlucose) > 50) || ((trendOneGlucose - lastGlucose) > 50))
          currentGlucose = trendTwoGlucose;
       else
          currentGlucose = trendOneGlucose;
    }

    for (int i=0; i<16; i++)
    {
      if (((lastGlucose - trend[i]) > 50) || ((trend[i] - lastGlucose) > 50)) // invalid trend check
         continue;
      else
      {
         validTrend[validTrendCounter] = trend[i];
         validTrendCounter++;
      }
    }

    if (validTrendCounter > 0)
    { 
      for (int i=0; i < validTrendCounter; i++)
         averageGlucose += validTrend[i];
         
      averageGlucose = averageGlucose / validTrendCounter;
      
      if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
         shownGlucose = averageGlucose; // If currentGlucose is still invalid take the average value
      else
         shownGlucose = currentGlucose; // All went well. Take and show the current value
    }
    else
      shownGlucose = currentGlucose; // If all is going wrong, nevertheless take and show a value 

    if ((lastGlucose == currentGlucose) && (sensorMinutesElapse > 21000)) // Expired sensor check
      noDiffCount++;

    if (lastGlucose != currentGlucose) // Reset the counter
      noDiffCount = 0;

    if (currentGlucose != 0)
      lastGlucose = currentGlucose; 
*/
    
    NFCReady = 2;
    FirstRun = 0;

    if (noDiffCount > 5)
      return 0;
    else  
      return shownGlucose;
    
    }
  else
    {
    Serial.print("Read Memory Block Command FAIL");
    NFCReady = 0;
    readError = 0;
    }
    return 0;
 }

float Glucose_Reading(unsigned int val) {
        int bitmask = 0x0FFF;
        return ((val & bitmask) / 8.5);
}

String Build_Packet(float glucose) {
  Serial.println("Build Packet");
  
// Let's build a String which xDrip accepts as a BTWixel packet

      unsigned long raw = glucose*1000; // raw_value
      String packet = "";
      packet = String(raw);
      packet += ' ';
      packet += "216";
      packet += ' ';
      packet += String(batteryPcnt);
      packet += ' ';
      packet += String(sensorMinutesElapse);
      Serial.print("Glucose level: ");
      Serial.println(glucose);
      /*
      Serial.println("15 minutes-trend: ");
      for (int i=0; i<16; i++)
      {
        Serial.println(String(trend[i]));
      }
      Serial.print("Battery level: ");
      Serial.print(batteryPcnt);
      Serial.println("%");
      */
      delay(100);
      Serial.print("Sensor lifetime: ");
      Serial.print(sensorMinutesElapse);
      Serial.println(" minutes elapsed");
      Serial.println("Packet sent : " + packet);
      return packet;
}

void Send_Packet(String packet) {
   if ((packet.substring(0,1) != "0"))
    {
#ifdef ESP32
      int Packet_Size = packet.length() + 1;
      char BlePacket[Packet_Size];
      packet.toCharArray(BlePacket, Packet_Size);
      pTxCharacteristic->setValue(BlePacket);
      pTxCharacteristic->notify();     
#endif      
      delay(500);
    }
   else
    {
      Serial.println("");
      Serial.print("Packet not sent! Maybe a corrupt scan or an expired sensor.");
      Serial.println("");
      delay(500);
    }
  }

void goToSleep() {
 
 SPI.end();
 digitalWrite(MOSIPin, LOW);
 digitalWrite(SCKPin, LOW);
 digitalWrite(NFCPin1, LOW); // Turn off all power sources completely
 digitalWrite(NFCPin2, LOW); // for maximum power save on BM019.
 digitalWrite(NFCPin3, LOW);
 digitalWrite(NFCPin4, LOW);
 digitalWrite(IRQPin, LOW);
#ifdef ESP32
 sleeptime = millis();
 esp_sleep_enable_timer_wakeup(SLEEP_TIME - (sleeptime-boottime)*1000);
 esp_deep_sleep_start();
#else
  delay(30000);

  digitalWrite(NFCPin1, HIGH);
  digitalWrite(NFCPin2, HIGH);
  digitalWrite(NFCPin3, HIGH);
  digitalWrite(NFCPin4, HIGH);

  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  SPI.begin();

  delay(10);                      // send a wake up
  digitalWrite(IRQPin, LOW);      // pulse to put the 
  delayMicroseconds(100);         // BM019 into SPI
  digitalWrite(IRQPin, HIGH);     // mode 
  delay(10);
  digitalWrite(IRQPin, LOW);
  NFCReady = 0;
#endif

}

void loop() {

batteryPcnt=100;
  
  if (NFCReady == 0)
  {
    SetProtocol_Command(); // ISO 15693 settings
    delay(100);
  }
  else if (NFCReady == 1)
  {
    for (int i=0; i<3; i++) {
      Inventory_Command(); // sensor in range?
      if (NFCReady == 2)
        break;
      delay(1000);
    }
    if (NFCReady == 1) {
      goToSleep();
    }
  }
  else
  {
    String xdripPacket = Build_Packet(Read_Memory());
    Send_Packet(xdripPacket);
    goToSleep();
  }
}
