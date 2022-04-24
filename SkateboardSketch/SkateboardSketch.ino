//========Libraries============
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
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
#define OLED_MOSI (3)
#define OLED_CLK (4)
#define OLED_DC (5)
#define OLED_CS (6)
#define OLED_RESET(7)

//LED Pin(s)
#define LEDDATA (12)
#define NUMLEDS (16)
//========Globals=============
//Ultrasonic range variable(s)
bool RangeError;

//Acceleromter + Gyroscope variable(s)
Adafruit_MPU6050 Mpu;

//OLED Screen
Adafruit_SSD1306 Display(SCREENWIDTH, SCREENHEIGHT),
                                        OLED_MOSI, OLED_CLK, 
                                        OLED_DC, OLED_RESET, OLED_CS;
//=========================
void setup() {
  //for serial monitor debugging
  Serial.begin(9600);

  //pin set up for ultrasonic range finders
  pinMode(PORTTRIGGER, OUTPUT);
  pinMode(STARECHO, INPUT);
  pinMode(STARRIGGER, OUTPUT);
  pinMode(PORTECHO, INPUT);

  //initialize the accelerometer
  if (!Mpu.begin())
    printDebug("Failed to find MPU6050 chip");

  Mpu.setAcceleromterRange(MPU6050_RANGE_16_G);
  Mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  //initialize the OLED Screen
  if (!Display.begin(SSD1306_SWITCHCAPVCC, 0x3C)
    printDebug("Failed to start display");

  Display.display();
  delay(500);
  Display.setTextSize(1);
  Display.setTextColor(WHITE);
  Display.setRotation(0);
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
  Display.clearDisplay();
  Display.setCursor(0, 0);
  Display.println("Accelerometer - m/s^2");
  Display.print(a.acceleration.x, 1);
  Display.print(", ");
  Display.print(a.acceleration.y, 1);
  Display.print(", ");
  Display.println(a.acceleration.z, 1);
  //----------------------------------------------------
}
//=======Helper Functions=======
void printDebug(String msg){
  Serial.println("[DEBUG]" + msg);
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
//=========================
