//========Libraries============
#include <Adafruit_MPU6050.h> //accelerometer
#include <Adafruit_Sensor.h>    //accelerometer
#include <Adafruit_NeoPixel.h>  //LED's
#include <Adafruit_SSD1306.h> //OLED Display
#include <SPI.h>                      //OLED Display
#include <Adafruit_GFX.h>       //OLED Display
#include <Wire.h>
//========Macros=============
//LED Pin(s)
#define LEDDATA (12)
#define NUMPIXELS (16)
#define DELAYVAL (500)  //time in ms to pause b/w pixels
//constants defining the ranges of LED's on either side
#define PORTSTART (0)
#define PORTEND (14)
#define STARSTART (0)
#define STAREND (14)
//realistic distance values (should be 0 - 255) to change LED colors
#define CLOSEST (10) //should be around 2 inches mapped to raw
#define STEP (5)      //how much of a difference the value has to have to change color
#define NSTEPS (9)  //how many colors are available
//========Globals=============
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

  //initialize the LED strip/Neopixel
  initPixelStrip();
}

void loop() {
  float starDist, portDist;
  String debugMsg;

  //----------------------------------------------------
  portDist = 10;
  starDist = 10;
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

void initPixelStrip(){
  PixelStrip.begin();
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
