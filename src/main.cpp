#include <Arduino.h>

#define ESP32_CAN_TX_PIN GPIO_NUM_27  // Set CAN TX port to 26  
#define ESP32_CAN_RX_PIN GPIO_NUM_26  // Set CAN RX port to 27

#include <Arduino.h>
#include <Preferences.h>
#include <NMEA2000_CAN.h>       
#include <N2kMessages.h>
#include <mcp_can.h>
#include <SPI.h>

typedef struct
{
  double runningHours;
  double waterTemperature;
  double voltage;
  double speed;
  double oilPressure;
  double oilTemperature;
} EngineData_type;

EngineData_type engineData = {.runningHours = 0, .waterTemperature = 0,.voltage = 0, .speed = 0};

int nodeAddress;            // To store last Node Address
Preferences preferences;    // Nonvolatile storage on ESP32 - To store LastDeviceAddress


const unsigned long transmitMessages[] PROGMEM = {127488L, 127489L,0}; // Set the information for other bus devices, which messages we support

#define CAN0_INT 17                            
MCP_CAN CAN0(5);                               

// forward declarations
void          SayHello(void);
void          CheckSourceAddressChange(void);


//*****************************************************************************
void setup() {
  uint8_t chipId = 0;
  uint32_t id = 0, i;

  Serial.begin(115200);
  delay(50);

  SayHello();  // print some useful information to USB-serial

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
  preferences.end();
  Serial.printf("\nN2K-nodeAddress=%d\n", nodeAddress);

// If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
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
  static int sendStat;
  bool recivedRPM = false;
  
  NMEA2000.ParseMessages();  // to be removed?

  CheckSourceAddressChange();

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
                      Serial.printf("Recived engine epeed: %.1f\n", engineData.speed);
                      recivedRPM = true;
                    }
                    break;
        case 65253: engineData.runningHours = ((dataIn[0] + dataIn[1] * 256.00)/20.00) * 3600.00;
                    Serial.printf("Recived engine hours: %.1f\n", engineData.runningHours / 3600);
                    break;
        case 65262: engineData.waterTemperature = dataIn[0] - 40;
                    engineData.oilTemperature = dataIn[3] * 256 + dataIn[2];
                    Serial.printf("Recived water temperature: %.1f\n", engineData.waterTemperature);
                    Serial.printf("Recived oil temperature: %.1f\n", engineData.oilTemperature);
                    Serial.printf("Recived oil temperature: %x  %x\n", dataIn[2], dataIn[3]);
                    break;
        case 65271: engineData.voltage = (dataIn[7] * 256.0 + dataIn[6]) / 20.0;
                    Serial.printf("Recived alternator voltage: %.1f\n", engineData.voltage);
                    break;
        case 65263: engineData.oilPressure = dataIn[3];
                    Serial.printf("Recived oil pressure: %u\n", engineData.oilPressure);
                    break;
        case 65226: Serial.printf("Recived diagnostic data: %x\n", dataIn[0]);
                    break;
      }
    }
    currentTime = millis();
  }
  N2kMsg = (unsigned char)61444;
  SetN2kPGN127488(N2kMsg, 0, (double) engineData.speed, (double) N2kDoubleNA, (int8_t) N2kInt8NA); // prepare the datagramm
  NMEA2000.SendMsg(N2kMsg);

  N2kMsg = (unsigned char)65253;
  //SetN2kPGN127489 (N2kMsg, 0, (unsigned long)200000, (unsigned long)293 , CToKelvin(engineData.waterTemperature), engineData.voltage, N2kDoubleNA, engineData.runningHours, N2kDoubleNA, N2kDoubleNA, N2kInt8NA, N2kInt8NA, 0xff, 0xff); //Test data
  SetN2kPGN127489 (N2kMsg, 0, engineData.oilPressure, engineData.oilTemperature, CToKelvin(engineData.waterTemperature), engineData.voltage, N2kDoubleNA, engineData.runningHours, N2kDoubleNA, N2kDoubleNA, N2kInt8NA, N2kInt8NA, 0x00,0x00);
  NMEA2000.SendMsg(N2kMsg);

  Serial.printf("Sent NMEA data\n");


} 

//*****************************************************************************
// Function to check if SourceAddress has changed (due to address conflict on bus)
void CheckSourceAddressChange() 
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

//*****************************************************************************
void SayHello()
{
  char Sketch[80], buf[256], Version[80];
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