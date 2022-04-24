//========Macros=============
//Ultrasonic Range Sensor Pin(s)
#define PORTTRIGGER (10)
#define STARTRIGGER (9)
#define PORTECHO (13)
#define STARTECHO (8)

//Acceleromter + Gyroscope Pin(s)
#define SCLPIN (A5)
#define SDAPIN (A4)

//LED Pin(s)
#define LEDDATA (12)
#define NUMLEDS (16)
//========Globals=============
//=========================
void setup() {
  //for serial monitor debugging
  Serial.begin(9600);

  //pin set up for ultrasonic range finders
  pinMode(PORTTRIGGER, OUTPUT);
  pinMode(STARTECHO, INPUT);
  pinMode(STARTRIGGER, OUTPUT);
  pinMode(PORTECHO, INPUT);
  
}

void loop() {
  ;
}
//=========================
