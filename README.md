# arduinoCodeShirt
code necessary to upload on the arduino embroidered in the shirt, to communicate with the android app. custom libraries are included and necessary to run the sketch.

Download the zip file from github and unzip it in a folder.
The arduino IDE is included in this zip file which is able to communicate with the arduino flora by adafruit.
The changed libraries are also included, if you want your own arduino IDE please copy the following libraries from this file

- Adafruit_Sensor
- adafruit LSM9DS0_Library
- WSWIRe
- adafruit AHRS


the arduino sketch can be found in the folder arduino sketches 

While doing some testing  I discovered that the arduino would freeze randomly after a few seconds of operation. 

The problem lies in the Wire Library. There are quite a few entries in wire.cpp that essentially read like this…

while (something != value) {
  something = grabValue
}
Basically, the possibility exists for the code to enter this while loop and never exit if something never equals value.

to solve this the custom WSWIRE library is used which adds a timeout to the while loops, so when something does not equal value after a given time the while loop will return an error message and the main loop can handle this error and do something else.
