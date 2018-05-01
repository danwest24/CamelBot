
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <NewPing.h>

#define DIR1 A5
#define DIR2 A4
#define PWM1 11
#define PWM2 12
#define DIR3 8
#define DIR4 7
#define PWM3 9
#define PWM4 10
#define PIEZO 13
#define BAT A3


//define sonar
#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

#define TRIG1 MOSI
#define TRIG2 SCK
#define TRIG3 A1

#define ECHO1 MOSI
#define ECHO2 SCK
#define ECHO3 A1

imu::Vector<3> orient;
imu::Vector<3> linAccel;
unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
const long interval = 100;
const long interval2 = 3000;
float systime;
int motor1State = 0;
int motor2State = 0;
int motor3State = 0;
int motor4State = 0;
bool flag = false;
char motorin[25];

int motorTempArray[4];
int beepTempArray[3];
int cnt = 0;
bool readflag = false;
int BATadc = 0;
float voltage = 0;
int sonarMeas[SONAR_NUM];
Adafruit_BNO055 bno = Adafruit_BNO055(55);
NewPing sonar[SONAR_NUM] = {   // Sensor object array - set pins and max distance of sensors

  NewPing(TRIG1, ECHO1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(TRIG2, ECHO2, MAX_DISTANCE),
  NewPing(TRIG3, ECHO3, MAX_DISTANCE),
  
  
};
void setup() {
  Serial.begin(9600);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(DIR4, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(PWM4, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PIEZO, OUTPUT);
  pinMode(BAT, INPUT);
  pinMode(ECHO1, OUTPUT);
  
  pinMode(ECHO3, INPUT);
  pinMode(ECHO2, INPUT);
  if (!bno.begin())
  {

    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1) {
      Serial.println("no IMU");
      delay(1000);
    }
  }
  sensor_t sensor;
  bno.getSensor(&sensor);
}
int set_motor(int motornum, int sp) { //Motor set function - sets given motor to given speed.
  // pass in the motor number (1 - 4) and the motor speed (-255 - 255) to set motor to appropriate speeds.
  //negative numbers for speed indicates opposite direction
  //1 PWM and 1 DIR pin is connected for each motor.

  if (motornum == 1) {
    motor1State = sp;
    if (sp < 0) {
      sp = -sp;
      digitalWrite(DIR1, LOW);
    }
    else {
      digitalWrite(DIR1, HIGH);
    }

    analogWrite(PWM1, sp);
    return (motor1State);
  }
  else if (motornum == 2) {
    motor2State = sp;
    if (sp < 0) {
      sp = -sp;
      digitalWrite(DIR2, LOW);
    }
    else {
      digitalWrite(DIR2, HIGH);
    }

    analogWrite(PWM2, sp);
    return (motor2State);
  }
  else if (motornum == 3) {
    motor3State = sp;
    if (sp < 0) {
      sp = -sp;
      digitalWrite(DIR3, LOW);
    }
    else {
      digitalWrite(DIR3, HIGH);

    }

    analogWrite(PWM3, sp);
    return (motor3State);
  }
  else if (motornum == 4) {
    motor4State = sp;
    if (sp < 0) {
      sp = -sp;
      digitalWrite(DIR4, LOW);
    }
    else {
      digitalWrite(DIR4, HIGH);
    }

    analogWrite(PWM4, sp);
    return (motor4State);
  }
  else {
    return (0);
  }

}

void Parse(char com[25]) { // Motor Parsing Function
  //This function is called whenever a full line of data is received from the Pi (denoted by the newline char).
  //When called, it parses the char array and decides which motor channel to power and at what speed/direction.
  //Input data from the Pi looks like : M1125N0231P1254Q1115 where MNPQ denote which channel. The first number is a bool
  //that represents direction (0 for backward, 1 for forward). The next three numbers are the speed of the motor, from
  //0 to 255. The input data is an array of char due to the nature of the serial input, and must first be translated from
  //ASCII code to the proper number by subtracting 48., then combining into an int type by multiplying the decimal places
  //by the appropriate factor.
  for (int i = 0; i < 23; i++) {
    int tmpval = 0;
    if (com[i] == 'B') {
      beepTempArray[0] = com[i + 1] - 48;
      beepTempArray[1] = com[i + 2] - 48;
      beepTempArray[2] = com[i + 3] - 48;
      tmpval = (beepTempArray[0] * 100 + beepTempArray[1] * 10 + beepTempArray[2]);
      if (tmpval == 0) {
        digitalWrite(PIEZO, LOW);
      }
      else {
        analogWrite(PIEZO, tmpval);
      }
    }
    if (com[i] == 'M') {
      motorTempArray[0] = com[i + 1] - 48;
      motorTempArray[1] = com[i + 2] - 48;
      motorTempArray[2] = com[i + 3] - 48;
      motorTempArray[3] = com[i + 4] - 48;
      (motorTempArray[1] * 100 + motorTempArray[2] * 10 + motorTempArray[3]);
      if (motorTempArray[0] == 0) {
        set_motor(1, -(motorTempArray[1] * 100 + motorTempArray[2] * 10 + motorTempArray[3]));
      }
      else {
        set_motor(1, (motorTempArray[1] * 100 + motorTempArray[2] * 10 + motorTempArray[3]));
      }
    }
    if (com[i] == 'N') {
      motorTempArray[0] = com[i + 1] - 48;
      motorTempArray[1] = com[i + 2] - 48;
      motorTempArray[2] = com[i + 3] - 48;
      motorTempArray[3] = com[i + 4] - 48;
      (motorTempArray[1] * 100 + motorTempArray[2] * 10 + motorTempArray[3]);
      if (motorTempArray[0] == 0) {
        set_motor(2, -(motorTempArray[1] * 100 + motorTempArray[2] * 10 + motorTempArray[3]));
      }
      else {
        set_motor(2, (motorTempArray[1] * 100 + motorTempArray[2] * 10 + motorTempArray[3]));
      }
    }
    if (com[i] == 'P') {
      motorTempArray[0] = com[i + 1] - 48;
      motorTempArray[1] = com[i + 2] - 48;
      motorTempArray[2] = com[i + 3] - 48;
      motorTempArray[3] = com[i + 4] - 48;
      if (motorTempArray[0] == 0) {
        set_motor(3, -(motorTempArray[1] * 100 + motorTempArray[2] * 10 + motorTempArray[3]));
      }
      else {
        set_motor(3, (motorTempArray[1] * 100 + motorTempArray[2] * 10 + motorTempArray[3]));
      }
    }
    if (com[i] == 'Q') {
      motorTempArray[0] = com[i + 1] - 48;
      motorTempArray[1] = com[i + 2] - 48;
      motorTempArray[2] = com[i + 3] - 48;
      motorTempArray[3] = com[i + 4] - 48;

      if (motorTempArray[0] == 0) {
        set_motor(4, -(motorTempArray[1] * 100 + motorTempArray[2] * 10 + motorTempArray[3]));
      }
      else {
        set_motor(4, (motorTempArray[1] * 100 + motorTempArray[2] * 10 + motorTempArray[3]));
      }
    }
  }
}
void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();
  while (Serial.available()) {
    char c = Serial.read();
    motorin[cnt++] = c;
    if ((c == '\n') || (cnt == sizeof(motorin) - 1))
    {
      motorin[cnt] = '\0';
      cnt = 0;
      readflag = true;
    }
  }
  if (readflag == true) {
    //here the parsing function will read the input and set motor values from it. should be in a null-terminated char buffer
    Serial.print(motorin);
    readflag = false;

    Parse(motorin);
  }
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    //ACQUIRING MEASUREMENT DATA

    //get system time
    systime = float(currentMillis) / 1000;
    linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    //grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    orient = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    BATadc = analogRead(BAT);
    voltage = BATadc * 0.01367;
    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
      sonarMeas[i] = sonar[i].ping_cm();
      
    }
    Serial.print("T");
    Serial.print(systime);
    Serial.print("B");
    Serial.print(voltage);
    
    Serial.print("Sl");
    Serial.print(sonarMeas[0]);
    Serial.print("Sc");
    Serial.print(sonarMeas[1]);
    Serial.print("Sr");
    Serial.print(sonarMeas[2]);

    Serial.print("Ox");
    Serial.print(orient.x());
    Serial.print("Oy");
    Serial.print(orient.y());
    Serial.print("Oz");
    Serial.print(orient.z());

    Serial.print("Ax");
    Serial.print(linAccel.x());
    Serial.print("Ay");
    Serial.print(linAccel.y());
    Serial.print("Az");
    Serial.print(linAccel.z());

    Serial.print("M");
    Serial.print(motor1State);
    Serial.print("N");
    Serial.print(motor2State);
    Serial.print("P");
    Serial.print(motor3State);
    Serial.print("Q");
    Serial.print(motor4State);
    Serial.println();
    
    //set_motor(3, 100);
    //set_motor(4, 100);
  }



}
