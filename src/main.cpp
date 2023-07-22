#define WIRELESS_MODE 1               //Comment this to disable OTA and WebSerial support

#define ESP32_CAN_TX_PIN GPIO_NUM_27  // Set CAN TX port to 27  
#define ESP32_CAN_RX_PIN GPIO_NUM_26  // Set CAN RX port to 26
#define NUMBER_OF_SPEED_READS_TO_DETERMINE_ENGINE_STATE 2

#include <Arduino.h>
#include <Preferences.h>
#include <NMEA2000_CAN.h>       
#include <N2kMessages.h>
#include <mcp_can.h>
#include <SPI.h>

#ifdef WIRELESS_MODE
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <WebSerial.h>

const char* ssid = "VolvoPenta";
const char* password = "12345678";
IPAddress local_IP(192, 168, 1, 100);
AsyncWebServer server(80);
#endif

typedef struct
{
  double runningHours;
  double waterTemperature;
  double waterTemperatureKelvin;
  double voltage;
  double speed;
  double oilPressure;
  double oilTemperature;
  bool started;
} EngineData_type;

EngineData_type engineData = {.runningHours = 0,
                              .waterTemperature = 0,
                              .waterTemperatureKelvin = 0,
                              .voltage = 0, .speed = 0,
                              .oilPressure = 0,
                              .oilTemperature = 0,
                              .started = false};

int nodeAddress;            // To store last Node Address
Preferences preferences;    // Nonvolatile storage on ESP32

unsigned int noOfNonZeroRpmRead = 0;
unsigned int noOfZeroRpmRead = 0;
unsigned int noOfShudownsDetected = 0;
unsigned int noOfNoTempReadings = 0;
bool gotTemperature;
const unsigned long transmitMessages[] PROGMEM = {127488L, 127489L,0}; // Set the information for other bus devices, which messages we support

#define CAN0_INT 17                            
MCP_CAN CAN0(5);                               

// forward declarations
void Say_Hello(void);
void Check_Source_Address_Change(void);
void Save_Engine_Hours(void);
void Debug_Print ( const char * format, ... );

void setup() {
  delay(2000);
  #ifdef WIRELESS_MODE
  Serial.print("Setting AP (Access Point)…");
  WiFi.softAP(ssid, password);    /*ESP32 wifi set in Access Point mode*/

  IPAddress IP = WiFi.softAPIP();  /*IP address is initialized*/
  Serial.print("AP IP address: ");
  Serial.println(IP);  /*Print IP address*/
  server.begin();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Lindh Technologies Volvo Penta Gateway.");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  WebSerial.begin(&server);          // Start WebSerial
  server.begin();
  Serial.println("HTTP server started");
  #endif

  uint8_t chipId = 0;

  Serial.begin(115200);
  delay(50);

  Say_Hello();  // print some useful information to USB-serial

  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
    Serial.println("MCP2515 Initialized Successfully");
  else
  {
    Serial.println("Error Initializing MCP2515...");
    while (1);
  }
  
  CAN0.setMode(MCP_LISTENONLY);   // Set operation mode to listen only. We don't want to write anything to the VP CAN-Bus
  pinMode(CAN0_INT, INPUT);       // Configuring pin for INT input

 
// NMEA2000 initialisation section
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(150);
  NMEA2000.SetN2kCANSendFrameBufSize(150);

// Generate unique number from chip id
  for(int i=0; i<17; i=i+8) {
	  chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
	}

// Set product information
  NMEA2000.SetProductInformation("1", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "Lindh Technologies Volvo Penta Gateay",  // Manufacturer's Model ID
                                 "v1.0 (7-7-2023)",  // Manufacturer's Software version code
                                 "HW v1.0" // Manufacturer's Model version
                                );
// Set device information
  NMEA2000.SetDeviceInformation((unsigned long)chipId, // Unique number. Use e.g. Serial number.
                                160, // Device function=Device that brings information from an engine used for propulsion onto the NMEA 2000 network
                                50,  // Device class=Propulsion
                                2002 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

  preferences.begin("nvs", false);                          // Open nonvolatile storage (nvs)
  nodeAddress = preferences.getInt("LastNodeAddress", 37);  // Read stored last nodeAddress, default 34
  engineData.runningHours = (double)preferences.getFloat("RunningHours", 0.0); // Read stored running hours, default 0.0
  preferences.end();
  Serial.printf("\nN2K-nodeAddress: %d\n", nodeAddress);
  Serial.printf("\nRunning hours: %.2f\n", engineData.runningHours);

// To also see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, nodeAddress);
  NMEA2000.ExtendTransmitMessages(transmitMessages);

  NMEA2000.Open();
  delay(200);
}

void loop() 
{
  long unsigned int PGN;
  unsigned char len = 0;
  unsigned char dataIn[16];
  tN2kMsg N2kMsg;
  bool recivedRPM = false;
  gotTemperature = false;
  
  NMEA2000.ParseMessages();  // to be removed?

  Check_Source_Address_Change();

  if ( Serial.available())   // Dummy to empty input buffer to avoid board to stuck with e.g. NMEA Reader
  {      
    Serial.read();
  }
  unsigned long currentTime = millis();
  unsigned long startTime = currentTime;

  while(currentTime < startTime + 500){
    if(!digitalRead(CAN0_INT))
    {
      CAN0.readMsgBuf(&PGN, &len, dataIn);

      PGN = (PGN>>8)&0xFFFF;

      switch(PGN)
      {
        case 61444: if (!recivedRPM){
                      engineData.speed = (dataIn[4] * 256.0 + dataIn[3] ) / 8.0;                                                                      // send it out
                      Debug_Print("Recived engine epeed: %.1f\n", engineData.speed);
                      recivedRPM = true;
                    }
                    break;
        case 65253: engineData.runningHours = ((dataIn[0] + dataIn[1] * 256.00)/20.00) * 3600.00;
                    Debug_Print("Recived engine hours: %.1f\n", engineData.runningHours / 3600);
                    break;
        case 65262: engineData.waterTemperature = dataIn[0] - 40;
                    engineData.oilTemperature = dataIn[3] * 256 + dataIn[2];
                    engineData.waterTemperatureKelvin = CToKelvin(engineData.waterTemperature);
                    Debug_Print("Recived water temperature: %.1f\n", engineData.waterTemperature);
                    Debug_Print("Recived oil temperature: %.1f\n", engineData.oilTemperature);
                    noOfNoTempReadings = 0;
                    gotTemperature = true;
                    break;
        case 65271: engineData.voltage = (dataIn[7] * 256.0 + dataIn[6]) / 20.0;
                    Debug_Print("Recived alternator voltage: %.1f\n", engineData.voltage);
                    break;
        case 65263: engineData.oilPressure = dataIn[3];
                    Debug_Print("Recived oil pressure: %.1f\n", engineData.oilPressure);
                    break;
        case 65226: Debug_Print("Recived diagnostic data: %x\n", dataIn[0]);
                    break;
      }
    }
    currentTime = millis();
  }
  if (engineData.speed > 0 && !engineData.started){
    noOfZeroRpmRead = 0;
    noOfNonZeroRpmRead += 1;
    if(noOfNonZeroRpmRead >= NUMBER_OF_SPEED_READS_TO_DETERMINE_ENGINE_STATE){
      engineData.started = true;
      noOfNonZeroRpmRead = 0;
      Debug_Print("Engine start detected\n");
      }
  }
  if (engineData.speed <= 0 && engineData.started) {
    noOfNonZeroRpmRead = 0;
    noOfZeroRpmRead += 1;
    if (noOfZeroRpmRead >= NUMBER_OF_SPEED_READS_TO_DETERMINE_ENGINE_STATE){
      engineData.started = false;
      Debug_Print("Engne shutdown detected, saving engine hours...\n");
      Save_Engine_Hours();
    }
  }
  if (!gotTemperature){
    noOfNoTempReadings += 1;
    if (noOfNoTempReadings >= 5){
      engineData.waterTemperature = N2kDoubleNA;
      engineData.waterTemperatureKelvin = N2kDoubleNA;
      noOfNoTempReadings = 0;
    }
  }

  N2kMsg = (unsigned char)61444;
  SetN2kPGN127488(N2kMsg, 0, engineData.speed, (double) N2kDoubleNA, (int8_t) N2kInt8NA); // prepare the datagramm
  NMEA2000.SendMsg(N2kMsg);

  N2kMsg = (unsigned char)65253;
  //SetN2kPGN127489 (N2kMsg, 0, (unsigned long)200000, (unsigned long)293 , CToKelvin(engineData.waterTemperature), engineData.voltage, N2kDoubleNA, engineData.runningHours, N2kDoubleNA, N2kDoubleNA, N2kInt8NA, N2kInt8NA, 0xff, 0xff); //Test data
  SetN2kPGN127489 (N2kMsg, 0, engineData.oilPressure, engineData.oilTemperature, engineData.waterTemperatureKelvin, engineData.voltage, N2kDoubleNA, engineData.runningHours, N2kDoubleNA, N2kDoubleNA, N2kInt8NA, N2kInt8NA, 0x00,0x00);
  NMEA2000.SendMsg(N2kMsg);

  //Debug_Print("Sent NMEA data\n");
  //WebSerial.printf("Sent NMEA data\n");
  Debug_Print("Sent NMEA2000 data\n");
  Debug_Print("Number of shutdowmns detected: %u\n", noOfShudownsDetected);
} 

// Function to check if SourceAddress has changed (due to address conflict on bus)
void Check_Source_Address_Change() 
{
  int SourceAddress = NMEA2000.GetN2kSource();

  if (SourceAddress != nodeAddress) { // Save potentially changed Source Address to NVS memory
    nodeAddress = SourceAddress;      // Set new Node Address (to save only once)
    preferences.begin("nvs", false);
    preferences.putInt("LastNodeAddress", SourceAddress);
    preferences.end();
    Serial.printf("Address Change: New Address=%d\n", SourceAddress);
  }
}

void Say_Hello()
{
  char Sketch[80], buf[256];
  int i;

  // Get source code filename
  strcpy(buf, __FILE__);
  i = strlen(buf);

  // remove path and suffix
  while (buf[i] != '.' && i >= 0)
    i--;
  buf[i] = '\0';
  while ( buf[i] != '\\' && i >= 0)
    i--;
  i++;
  strcpy(Sketch, buf + i);

  // Sketch date/time of compliation
  sprintf(buf, "\nSketch: \"%s\", compiled %s, %s\n", Sketch, __DATE__, __TIME__);
  Serial.println(buf);
}

void Save_Engine_Hours()
{
  preferences.begin("nvs", false);
  preferences.putFloat("RunningHours", (float)engineData.runningHours);
  preferences.end();
  Debug_Print("Engine shutdown detetected, running hours saved: %.2f\n", engineData.runningHours);
  noOfShudownsDetected += 1;
}

void Debug_Print ( const char * format, ... )
{
  char *buffer;

  buffer = (char *)malloc(sizeof(char)*512);
  va_list args;
  va_start (args, format);
  vsnprintf (buffer,512,format, args);
  Serial.print(buffer);
  #ifdef WIRELESS_MODE
  WebSerial.print(buffer);
  #endif
  va_end (args);
  free(buffer);
}