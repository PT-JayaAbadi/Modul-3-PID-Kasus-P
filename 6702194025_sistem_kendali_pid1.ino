//============================================//

// PRAKTIKUM MODUL 4 PID Kasus P //
// Nama : Dewa Ramadhan Pradana - 6702194025 

//============================================//

int sensor1 = A0;
int sensor2 = A1;
int sensor3 = A2;
int sensor4 = A3;
int baca_sensor[4];

//Direktori Motor
#define CW  0
#define CCW 1

//Motor Control PIN
#define motorDirPin 7
#define motorPWMPin 9
#define enablePin   8
 
//PIN Encoder
#define encoderPinA 2
#define encoderPinB 4
 
//VAR Encoder
int encoderPos = 0;
 
//Control P
float Kp          = 1;
int   targetPos   = 100;
int   error;
int   control;
int   velocity;
 
//bacaan Encoder
void doEncoderA(){
  digitalRead(encoderPinB)?encoderPos--:encoderPos++;
}
 
void setup(){
  //Interrupt Setup
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA,RISING);

    //Setup Motor Driver
    pinMode(motorDirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, HIGH);

    Serial.begin(9600);
}
 
void loop(){
  error = targetPos - encoderPos;
  control = Kp * error;
  velocity = min(max(control, -255), 255);

  if(velocity >= 0){
    digitalWrite(motorDirPin, CW);
    analogWrite(motorPWMPin, velocity);

  }
  else{
    digitalWrite(motorDirPin, CCW);
    analogWrite(motorPWMPin, 255+velocity);

  }
  Serial.println(encoderPos);
}
  
//membaca sensor
void readsensor(){
readsensor();
  
baca_sensor[0] = analogRead(sensor1);
baca_sensor[1] = analogRead(sensor2);
baca_sensor[2] = analogRead(sensor3);
baca_sensor[3] = analogRead(sensor4);
  
//sensor 1 && 2
if (baca_sensor[0] < 34 && baca_sensor[1] < 34 && 
    baca_sensor[2] > 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34);
  
//sensor 2 && 3
if (baca_sensor[0] > 34 && baca_sensor[1] < 34 && 
    baca_sensor[2] < 34 && baca_sensor[3] > 34 && 
    baca_sensor[4] > 34);
  
//sensor 3 && 4
if (baca_sensor[0] > 34 && baca_sensor[1] > 34 && 
    baca_sensor[2] < 34 && baca_sensor[3] < 34 && 
    baca_sensor[4] > 34);  
}