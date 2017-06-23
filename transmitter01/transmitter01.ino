// This sketch is based on the Sparkfun RFM69HCW Example Sketch
#include <RFM69.h>
#include <SPI.h>
#include <BMI160.h>
#include <CurieIMU.h>

// Addresses for this node. CHANGE THESE FOR EACH NODE!

#define NETWORKID     0   // Must be the same for all nodes (0 to 255)
#define MYNODEID      2   // My node ID (0 to 255)
#define TONODEID      1   // Destination node ID (0 to 254, 255 = broadcast)

// RFM69 frequency, uncomment the frequency of your module:
#define FREQUENCY     RF69_915MHZ

// Data transmit state activated LED
#define TRASNMITLED           8 // LED positive pin

// FlightDataComputer state variables
unsigned long timeNow, timePrevious, transmitInterval;
boolean dataTransmit;

struct FlightData 
{
  long timePoint;
  float a[3];
  float g[3];  
};

union 
{
  struct FlightData flightData;
  unsigned char bytes[28];
} dataPacket; 

// Flight Data Computer Sensors
class IMU 
{
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;

  long timeNow, timePrevious, imuInterval;
  
  public:

  IMU(int dummy){}
  void init()
  {
    imuInterval = 10;
    CurieIMU.begin();
    CurieIMU.setGyroRate(25);
    CurieIMU.setAccelerometerRate(25);
    CurieIMU.setAccelerometerRange(2);
    CurieIMU.setGyroRange(250);
    timeNow = 0;
    timePrevious = 0;
  }
  void Update()
  {
    timeNow = millis();
    if ((timeNow - timePrevious) > imuInterval) 
    {
      timePrevious = timeNow;
      CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
      // convert from raw data to gravity and degrees/second units
      ax = convertRawAcceleration(aix);
      ay = convertRawAcceleration(aiy);
      az = convertRawAcceleration(aiz);
      gx = convertRawGyro(gix);
      gy = convertRawGyro(giy);
      gz = convertRawGyro(giz);
      dataPacket.flightData.timePoint = timeNow;
      dataPacket.flightData.a[0] = ax;
      dataPacket.flightData.a[1] = ay;
      dataPacket.flightData.a[2] = az;
      dataPacket.flightData.g[0] = gx;
      dataPacket.flightData.g[1] = gy;
      dataPacket.flightData.g[2] = gz; 
    }
  }
  
  private:
  // these functions taken from the CurieIMU demo libraries
  float convertRawAcceleration(int aRaw) 
  {
    // since we are using 2G range
    // -2g maps to a raw value of -32768
    // +2g maps to a raw value of 32767
  
    float a = (aRaw * 2.0) / 32768.0;
    return a;
  }

  float convertRawGyro(int gRaw) 
  {
    // since we are using 250 degrees/seconds range
    // -250 maps to a raw value of -32768
    // +250 maps to a raw value of 32767
  
    float g = (gRaw * 250.0) / 32768.0;
    return g;
  }
};

// Create a library object for our RFM69HCW module:
struct FlightData flightData;
RFM69 radio;
IMU imu(1);
int commandCode;

void setup()
{
  Serial.begin(9600);
  commandCode = -1;
  imu.init();
  
  timeNow = 0;
  timePrevious = 0;
  transmitInterval = 100; // 3 secons in milliseconds for testing purposes
  // Initialize the RFM69HCW:
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW
}

void loop()
{
  imu.Update();
  if (radio.receiveDone()) // Got one
  {
    commandCode = (int)radio.DATA[0];     
    switch(commandCode)
    {
      case 0:
        break;
      case 1:
        dataTransmit = TRUE;
        break;
      case 2:
        dataTransmit = FALSE;
        break;
      case 3:
        break;
      case 4:
        break;
      default:
        break;
    }
  }
  timeNow = millis();
  if ( ((timeNow - timePrevious) > transmitInterval) && dataTransmit) 
  {
    timePrevious = timeNow;  
    radio.send(TONODEID, dataPacket.bytes, sizeof(dataPacket.bytes));
    Serial.println("Transmitting Data");     
  }
}

