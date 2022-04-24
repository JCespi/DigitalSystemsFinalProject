//========Macros=============
//Ultrasonic Range Sensor Pin(s)
#define PORTTRIGGER (10)
#define STARRIGGER (9)
#define PORTECHO (13)
#define STARECHO (8)

//Acceleromter + Gyroscope Pin(s)
#define SCLPIN (A5)
#define SDAPIN (A4)

//LED Pin(s)
#define LEDDATA (12)
#define NUMLEDS (16)
//========Globals=============
//Ultrasonic range variable(s)
bool RangeError;
//=========================
void setup() {
  //for serial monitor debugging
  Serial.begin(9600);

  //pin set up for ultrasonic range finders
  pinMode(PORTTRIGGER, OUTPUT);
  pinMode(STARECHO, INPUT);
  pinMode(STARRIGGER, OUTPUT);
  pinMode(PORTECHO, INPUT);
  
}

void loop() {
  float starDist, portDist;
  String debugMsg;

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
