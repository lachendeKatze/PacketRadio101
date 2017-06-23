// based on Sparkfun RFM69HCW Example Sketch

#include <RFM69.h>
#include <SPI.h>
#include <MadgwickAHRS.h>

class CommandConsole 
{
  #define TRANSMITBUTTON 7
  #define TRANSMITINDICATOR 5
  
  boolean dataTransmit,dataTransmitButton,dataTransmitPressed,dataTransmitReleased;
  long timeNow, timePrevious, debounceInterval;

  public:

    CommandConsole(int dummy){}
    void init() 
    {
      pinMode(TRANSMITBUTTON, INPUT_PULLUP);
      pinMode(TRANSMITINDICATOR,OUTPUT);
      digitalWrite(TRANSMITINDICATOR,LOW);
      dataTransmit = FALSE;
      dataTransmitButton = TRUE;
      dataTransmitPressed = FALSE;
      timeNow = 0;
      timePrevious = 0;
      debounceInterval = 5;
    }
    
    void Update() 
    {
      
      timeNow = millis();
      if ((timeNow-timePrevious)>debounceInterval)
      {
        timePrevious = timeNow;
        dataTransmitPressed = test_for_press_only();
        updateDisplay();
      }
      // Serial.print("Pressed: ");Serial.println(dataTransmitPressed);
    }

    boolean getDataTransmit() {return dataTransmit;}
    boolean getDataTransmitPressed() {return dataTransmitPressed;}
    
  private:

    uint8_t test_for_press_only(void)
    {
      static uint8_t button_history = 0;
      uint8_t pressed = 0;    
 
      button_history = button_history << 1;
      button_history |= read_button();
      if ((button_history & 0b11000111) == 0b00000111)
      { 
          pressed = 1;
          button_history = 0b11111111;
      }
      return pressed;
    }
   
    boolean read_button() { return digitalRead(TRANSMITBUTTON) == 0; }
    
    void updateDisplay()
    {
      if (dataTransmitPressed) 
      { 
        digitalWrite(TRANSMITINDICATOR,HIGH); 
      }
      else { 
        digitalWrite(TRANSMITINDICATOR,LOW);
      }
    }
};


// Addresses for this node. CHANGE THESE FOR EACH NODE!

#define NETWORKID     0   // Must be the same for all nodes (0 to 255)
#define MYNODEID      1   // My node ID (0 to 255)
#define TONODEID      2   // Destination node ID (0 to 254, 255 = broadcast)

uint8_t transmitCode[1];
uint8_t stopCode[1]; 

// RFM69 frequency, uncomment the frequency of your module:
#define FREQUENCY     RF69_915MHZ

// Create a library object for our RFM69HCW module:
Madgwick filter;
RFM69 radio;
CommandConsole cc(1);
boolean transmit, transmittingData;
float roll, pitch, heading;



struct FlightData 
{
  long timePoint;
  float a[3];
  float g[3];  
};

union 
{
  struct FlightData flightData;
  unsigned char bytes[61];
} dataPacket; 


void setup()
{
  Serial.begin(9600);
  transmit = FALSE;
  transmittingData= FALSE;
  transmitCode[0] = {1};
  stopCode[0] = {2};
  // Initialize the RFM69HCW:
  
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW

  cc.init();
  filter.begin(25);
}



void loop()
{
  // RECEIVING

  // In this section, we'll check with the RFM69HCW to see
  // if it has received any packets:

  cc.Update();
  transmit = cc.getDataTransmitPressed();

  if(transmit && !transmittingData) 
  { 
    Serial.println("Start Data Tx");
    radio.send(TONODEID,transmitCode, sizeof(transmitCode));
    transmittingData = TRUE;
  }
  else if (transmit && transmittingData)
  {
    Serial.println("Stop Data Tx");
    radio.send(TONODEID,stopCode, sizeof(stopCode));
    transmittingData = FALSE;
  }
  
  if (radio.receiveDone()) // Got one!
  {
    for (byte i = 0; i < radio.DATALEN; i++)
    {
      // Serial.print(radio.DATA[i]);Serial.print(" "); 
      dataPacket.bytes[i] = radio.DATA[i];   
    }
  }
  
    filter.updateIMU(dataPacket.flightData.g[0], dataPacket.flightData.g[1], dataPacket.flightData.g[2], dataPacket.flightData.a[0], dataPacket.flightData.a[1], dataPacket.flightData.a[2]);
    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
    
 
  // Serial.print("Flight Data: ");Serial.print("time: ");Serial.print(dataPacket.flightData.timePoint);
  // Serial.print(" AccData: ");Serial.print(dataPacket.flightData.a[0]);Serial.print(" ");Serial.print(dataPacket.flightData.a[1]);Serial.print(" ");Serial.print(dataPacket.flightData.a[2]);
  // Serial.print(" GyroData: ");Serial.print(dataPacket.flightData.g[0]);Serial.print(" ");Serial.print(dataPacket.flightData.g[1]);Serial.print(" ");Serial.print(dataPacket.flightData.g[2]);
  // Serial.println();
  // Serial.println(cc.getDataTransmit());
}

