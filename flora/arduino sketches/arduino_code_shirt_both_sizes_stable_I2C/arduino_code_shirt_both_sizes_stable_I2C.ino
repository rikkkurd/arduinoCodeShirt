#include <SPI.h>
//#include <wire.h> // don't use the regular i2c library, this will freeze the arduino with unstable connections due to missing time-out function
#include <WSWire.h> // use this I2C library instead
#include <Adafruit_Sensor.h> // stable readings accross sensors
#include <Adafruit_LSM9DS0.h> // library to communicate with the orientation sensors (accelelerometer, gyroscope, and magnetometer)
#include <Adafruit_Simple_AHRS.h> //convert raw sensor data to orientation data yaw pitch heading
//#include <PacketSerial.h> // to send serial packets, not currently used in this sketch


// Create LSM9DS0 board instance.
Adafruit_LSM9DS0     lsm(1000);  // Use I2C, ID #1000

// Create simple AHRS algorithm using the LSM9DS0 instance's accelerometer and magnetometer.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());

// Function to configure the sensors on the LSM9DS0 board.
// You don't need to change anything here, but have the option to select different
// range and gain values.
void configureLSM9DS0(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}
int selectPin = 12; // NX3L1T3157 switch is used to select sensor 1 or sensor 2 make this pin high or low to change connected sensor
int values[8];


//PacketSerial serial;
/*
we connect 2 sensors with the same I2C address to the same arduino, to prevent communication clashes a switch connected to selectPin is used to 
switch the dataline connection (SDA) from one sensor to the other, this means only one sensor at a time is connected to the SDA pin on the arduino(or other microcontroller)
in the setup we inititialize each sensor by setting the selectPin Low for the first sensor, 
then initialize and after this we set the selectPin HIGH to initialize the second sensor.
furthermore we initialize Serial communication on Serial 1 instead of standart Serial.begin to communicate with the bluetooth chip on pin RX/TX instead of the usb cable


*/

void setup()
{
  delay(2000);
  pinMode(selectPin, OUTPUT);
//Serial.begin(9600);
  serial.begin(9600,1); // packet serial initialization baudrate 9600 - 1 means that Serial1.print is used by the library (see packetSerial library)
//  Serial.begin(9600); // normal wired serial port connection user Serial.println(" data " ); for output via wired serial connection
 delay(1000);
  digitalWrite(selectPin, LOW); // connect sensor 1 via the NX3L1T3157 switch chip
delay(1000);
  // Initialise the LSM9DS0 board.
  if (!lsm.begin())
  {
    // There was a problem detecting the LSM9DS0 ... check your connections

    while (1);
  }

  // Setup the sensor gain and integration time.
  configureLSM9DS0();


delay(300);
digitalWrite(selectPin, HIGH); // connect sensor 2 via the NX3L1T3157 switch chip
delay(200);
// Initialise the LSM9DS0 board.
if (!lsm.begin())
{
  // There was a problem detecting the LSM9DS0 ... check your connections

  while (1);
}

// Setup the sensor gain and integration time.
configureLSM9DS0();
}

void loop(void)
{
   delay(250);
    digitalWrite(selectPin, HIGH); // connect sensor 1 via the NX3L1T3157 switch chip
delay(250);

  sensors_vec_t   orientation;

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    
    
    
    values[0] = 0; // identifier for sensor 1
    values[1] = orientation.roll;
    values[2] = orientation.pitch;
    values[3] = orientation.heading;
    // send values to bluetooth chip
    Serial1.print(values[0]);
   Serial1.print(",");
   Serial1.print(values[1]);
     Serial1.print(",");
   Serial1.print(values[2]);
     Serial1.print(",");
   Serial1.print(values[3]);
      
   }
delay(250);
     digitalWrite(selectPin, LOW); // connect sensor 1 via the NX3L1T3157 switch chip
delay(250);
  
    if (ahrs.getOrientation(&orientation))
  {
    values[4] = 1; // identifier for sensor 2
    values[5] = orientation.roll;
    values[6] = orientation.pitch -4;
    values[7] = orientation.heading;
     Serial1.print(values[4]);
   Serial1.print(",");
   Serial1.print(values[5]);
     Serial1.print(",");
   Serial1.print(values[6]);
     Serial1.print(",");
   Serial1.print(values[7]);


// this function prints the sensor values also on the usb port, for debugging purposes
  debugger();

 
}

void debugger(){
  
        Serial.print(values[0]);
   Serial.print(",");
   Serial.print(values[1]);
     Serial.print(",");
   Serial.print(values[2]);
     Serial.print(",");
   Serial.print(values[3]);
   Serial.print("....");
     Serial.print(values[4]);
   Serial.print(",");
   Serial.print(values[5]);
     Serial.print(",");
   Serial.print(values[6]);
     Serial.print(",");
   Serial.println(values[7]);
   
}
