//========Libraries============
#include <Adafruit_MPU6050.h> //accelerometer
#include <Adafruit_Sensor.h>    //accelerometer
#include <Adafruit_NeoPixel.h>  //LED's
#include <Adafruit_SSD1306.h> //OLED Display
#include <SPI.h>                      //OLED Display
#include <Adafruit_GFX.h>       //OLED Display
#include <Wire.h>
//========Macros=============
//Ultrasonic Range Sensor Pin(s)
#define PORTTRIGGER (10)
#define STARRIGGER (9)
#define PORTECHO (13)
#define STARECHO (8)

//LED Pin(s)
#define STARBOARDDATAPIN (12)
#define PORTDATAPIN (11)
#define NUMPIXELS (16)
#define DELAYVAL (500)  //time in ms to pause b/w pixels
//constants defining the ranges of LED's on either side
#define PORTSTART (0)
#define PORTEND (15)
#define STARSTART (0)
#define STAREND (15)
//realistic distance values (should be 0 - 255) to change LED colors
#define CLOSEST (10) //should be around 2 inches mapped to raw
#define STEP (2)      //how much of a difference the value has to have to change color
#define NSTEPS (9)  //how many colors are available
//========Globals=============
//Ultrasonic range variable(s)
bool RangeError;

//LED Strip
Adafruit_NeoPixel StarPixelStrip(NUMPIXELS, STARBOARDDATAPIN, 
                                           NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel PortPixelStrip(NUMPIXELS, PORTDATAPIN,
                                          NEO_GRB + NEO_KHZ800);
const unsigned int IntensityMatrix[NSTEPS][3] = {
  {204, 0, 0}, //CLOSEST + 0 * STEP = 10 | dark red
  {255, 128, 0}, //CLOSEST + 1 * STEP = 12 | red
  {255, 51, 51},//CLOSEST + 2 * STEP = 14 | light red
  {255, 128, 0}, //CLOSEST + 3 * STEP = 16 | dark orange
  {255, 153, 51}, //CLOSEST + 4 * STEP = 18 | orange
  {255, 178, 102}, //CLOSEST + 5 * STEP = 20  | light orange
  {0, 153, 0}, //CLOSEST + 6 * STEP = 22 | darkest green
  {0, 204, 0}, //CLOSEST + 7 * STEP = 24 | darker green
  {0, 255, 0} //CLOSEST + 8 * STEP = 26 > | green
};
//=========================
void setup() {
  //for serial monitor debugging
  Serial.begin(9600);

  //pin set up for ultrasonic range finders
  initRangeSensors();

  //initialize the LED strip/Neopixel
  initStrips();
}

void loop() {
  float starDist, portDist;
  String debugMsg;
  sensors_event_t a, g, temp;

  //----------------------------------------------------
  //get the distance on the starboard and port sides of the board
  portDist = getDistanceToSensor(true);
  starDist= getDistanceToSensor(false);
  
  //check for an out-of-bounds range
  debugMsg = "Port value = " + String(portDist) + " , starboard value = " + String(starDist);
  printDebug(debugMsg);
  if (RangeError)
    return;
  //----------------------------------------------------
  //write to the pixel strip
  showPixelResponse(portDist, PORTSTART, PORTEND);
  showPixelResponse(starDist, STARSTART, STAREND);
  //----------------------------------------------------
}
//=======Helper Functions=======
void printDebug(String msg){
  Serial.println("[DEBUG]" + msg);
}

void initRangeSensors(){
  pinMode(PORTTRIGGER, OUTPUT);
  pinMode(STARECHO, INPUT);
  pinMode(STARRIGGER, OUTPUT);
  pinMode(PORTECHO, INPUT);
}

void initStrips(){
  StarPixelStrip.begin();
  PortPixelStrip.begin();
}

unsigned int getDistanceToSensor(bool portSide){
  float duration, distance;
  unsigned int triggerPin, echoPin;

  //assign the appropriate pins
  triggerPin = (portSide == true) ? PORTTRIGGER : STARRIGGER;
  echoPin = (portSide == true) ? PORTECHO : STARECHO;

  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) * 0.0344;

  if (distance >= 400 || distance <= 2){
    RangeError = true;
    printDebug("Raw distance value " + String(distance) + " out of range");
  } else {
    RangeError = false;

    //convert to analog-friendly value
    map(distance, 2, 400, 0, 255);
    
    delay(500);
  }
  delay(500);
  
  return distance;
}

void showPixelResponse(bool portSide, float distance, unsigned int s, unsigned int e){
  unsigned int red, green, blue;

  if (portSide)
    PortPixelStrip.clear();
  else
    StarPixelStrip.clear();

  //based on the range in which the distance exists, assign a color
  for (int k=0; k < NSTEPS; k++){
    if (distance < CLOSEST + k * STEP){
      red = IntensityMatrix[k][0];
      green = IntensityMatrix[k][1];
      blue = IntensityMatrix[k][2];
      break;
    }
  }

  for (int pixel=s; pixel < e; pixel++){
    if (portSide){
      PortPixelStrip.setPixelColor(pixel, PortPixelStrip.Color(red, green, blue));
      PortPixelStrip.show();
    } else {
      StarPixelStrip.setPixelColor(pixel, StarPixelStrip.Color(red, green, blue));
      StarPixelStrip.show();
    }
  }
}
//=========================
