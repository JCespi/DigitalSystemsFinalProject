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

//Acceleromter + Gyroscope Pin(s)
#define SCLPIN (A5)
#define SDAPIN (A4)

//OLED Screen Pin(s)
#define SCREENWIDTH (128)
#define SCREENHEIGHT (64)
// Declaration for SSD1306 display connected using software SPI (default case):
#define OLEDMOSI (3)
#define OLEDCLK (4)
#define OLEDDC (5)
#define OLEDCS (6)
#define OLEDRESET (7)

//LED Pin(s)
#define LEDDATA (12)
#define NUMPIXELS (16)
#define DELAYVAL (500)  //time in ms to pause b/w pixels
//constants defining the ranges of LED's on either side
#define PORTSTART (0)
#define PORTEND (0)
#define STARSTART (0)
#define STAREND (0)
//realistic distance values (should be 0 - 255) to change LED colors
#define CLOSEST (10) //should be around 2 inches mapped to raw
#define STEP (5)      //how much of a difference the value has to have to change color
#define NSTEPS (9)  //how many colors are available
//========Globals=============
//Ultrasonic range variable(s)
bool RangeError;

//Acceleromter + Gyroscope variable(s)
Adafruit_MPU6050 Mpu;

//OLED Screen
Adafruit_SSD1306 Display(SCREENWIDTH, SCREENHEIGHT,
                                       OLEDMOSI, OLEDCLK, 
                                       OLEDDC, OLEDRESET, OLEDCS);

//LED Strip
Adafruit_NeoPixel PixelStrip(NUMPIXELS, LEDDATA, 
                                           NEO_GRB + NEO_KHZ800);
const unsigned int IntensityMatrix[NSTEPS][3] = {
  {0, 204, 0},
  {0, 153, 0},
  {76, 153, 0},
  {204, 204, 0},
  {255, 153, 51},
  {255, 128, 0},
  {204, 102, 0},
  {204, 204, 0},
  {255, 0, 0}
};
//=========================
void setup() {
  //for serial monitor debugging
  Serial.begin(9600);

  //pin set up for ultrasonic range finders
  initRangeSensors();

  //initialize the accelerometer
  initAccelerometer();
  delay(100);

  //initialize the OLED Screen
  initDisplay();

  //initialize the LED strip/Neopixel
  initPixelStrip();
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
  //get the new sensor events
  Mpu.getEvent(&a, &g, &temp);

  //show the sensor event(s)
  displayData(a, g, temp);
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

void initAccelerometer(){
  if (!Mpu.begin())
    printDebug("Failed to find MPU6050 chip");

  Mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  Mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void initDisplay(){
  if (!Display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    printDebug("Failed to start display");

  Display.display();
  delay(500);
  Display.setTextSize(1);
  Display.setTextColor(WHITE);
  Display.setRotation(0);
}

void initPixelStrip(){
  PixelStrip.begin();
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

void displayData(sensors_event_t a, sensors_event_t g, sensors_event_t temp){
  Display.clearDisplay();
  Display.setCursor(0, 0);
  Display.println("Accelerometer - m/s^2");
  Display.print(a.acceleration.x, 1);
  Display.print(", ");
  Display.print(a.acceleration.y, 1);
  Display.print(", ");
  Display.println(a.acceleration.z, 1);
}

void showPixelResponse(float distance, unsigned int s, unsigned int e){
  unsigned int red, green, blue;
  
  PixelStrip.clear();

  //based on the range in which the distance exists, assign a color
  for (int k=0; k < NSTEPS; k++){
    if (distance < CLOSEST + k * STEP){
      red = IntensityMatrix[k][0];
      green = IntensityMatrix[k][1];
      blue = IntensityMatrix[k][2];
    }
  }

  for (int pixel=s; pixel < e; pixel++){
    PixelStrip.setPixelColor(pixel, PixelStrip.Color(red, green, blue));
    PixelStrip.show();
  }
}
//=========================
