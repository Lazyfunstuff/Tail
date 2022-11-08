#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Keypad.h"
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int servoCenter[8];
int servoTarget[8];
int servoCurrent[8];
const byte Rows=3;
const byte Cols=3;
bool active = false;
String task;
char keyPadTask;
bool keyPadNewTask = false;
char keymap[Rows][Cols]={
  {'4','0','3'},
  {'6','1','5'},
  {'8','2','7'}
};
byte rPins[Rows]= {7,6,5}; //Rows 0 to 2
byte cPins[Cols]= {4,3,2}; //Columns 0 to 2
Keypad kpd= Keypad(makeKeymap(keymap), rPins, cPins, Rows, Cols);
int greenLED = 8;
int redLED = 9;

void setup() {
  Serial.begin(9600);
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  //center possition tweeking, ordered tip to root
  servoCenter[0] = 280;
  servoCenter[1] = 305;
  servoCenter[2] = 300;
  servoCenter[3] = 260;
  servoCenter[4] = 300;
  servoCenter[5] = 315;
  servoCenter[6] = 270;
  servoCenter[7] = 300;
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(10);
  cernterServos();
  pwm.sleep();
  digitalWrite(redLED, HIGH);
  printMenu();
}

void setServo(int servo, int target){
  int newTarget;
  //since every servo is rotated 90 degrees adjust so directions make sense
  if (servo == 0 || servo == 3 || servo == 4 || servo == 7){
    newTarget = servoCenter[servo] + target;
  }
  else {
    newTarget = servoCenter[servo] - target;
  }
  pwm.setPWM(servo, 0, newTarget);
  servoCurrent[servo] = target;
}

void cernterServos() {
  Serial.println("Center all servos - Warning");
  for (int i=0; i<=7; i++ ){
    pwm.setPWM(i, 0, servoCenter[i]);
    servoTarget[i] = 0;
    servoCurrent[i] = 0;
    Serial.print("center servo: ");
    Serial.println(i);
  }
}

bool checkArray(int arrayA[],int arrayB[]) {
  bool same = true;
  int numItems = 7;
  int i = 0;
  while(i<numItems && same) { 
    same = arrayA[i] == arrayB[i];
    i++;
  }
  return same;
}

void goToTarget(int d){
  while ( checkArray(servoCurrent, servoTarget) == false && !keyPad()){
    for (int i=0; i<=7; i++ ){
      if (servoCurrent[i] == servoTarget[i]){
      }else{
        if (servoCurrent[i] < servoTarget[i]){ servoCurrent[i]++; }
        else { servoCurrent[i]--; }
        setServo(i, servoCurrent[i]);
        delay(d);
      }
    }
  }
}

void wagTailLow(int goSpeed){
  Serial.print("Waging tail low at speed ");
  Serial.println(goSpeed);
  keyPad();
  while (!Serial.available() && !keyPad() && !keyPadNewTask) {
    servoTarget[0] = 20;
    servoTarget[1] = 110;
    servoTarget[2] = 30;
    servoTarget[3] = 80;
    servoTarget[4] = 30;
    servoTarget[5] = 80;
    servoTarget[6] = 30;
    servoTarget[7] = 40;
    goToTarget(goSpeed);
    delay(100);
    servoTarget[0] = 20;
    servoTarget[1] = -110;
    servoTarget[2] = 30;
    servoTarget[3] = -80;
    servoTarget[4] = 30;
    servoTarget[5] = -80;
    servoTarget[6] = 30;
    servoTarget[7] = -40;
    goToTarget(goSpeed);
    delay(100);
  }
}

void tailDown(int goSpeed){
  Serial.println("Tail down");
  servoTarget[0] = 0;
  servoTarget[1] = 0;
  servoTarget[2] = 0;
  servoTarget[3] = 0;
  servoTarget[4] = 0;
  servoTarget[5] = 0;
  servoTarget[6] = 0;
  servoTarget[7] = 0;
  goToTarget(goSpeed);
}

void wagTailHigh(int goSpeed){
  Serial.print("Waging tail high at speed ");
  Serial.println(goSpeed);
  keyPad();
  while (!Serial.available() && !keyPad() && !keyPadNewTask) {
    servoTarget[0] = 60;
    servoTarget[1] = 0;
    servoTarget[2] = 50;
    servoTarget[3] = 60;
    servoTarget[4] = 110;
    servoTarget[5] = 60;
    servoTarget[6] = 110;
    servoTarget[7] = 60;
    goToTarget(goSpeed);
    delay(200);
    servoTarget[0] = 60;
    servoTarget[1] = 0;
    servoTarget[2] = 50;
    servoTarget[3] = -60;
    servoTarget[4] = 110;
    servoTarget[5] = -60;
    servoTarget[6] = 110;
    servoTarget[7] = -60;
    goToTarget(goSpeed);
    delay(200);
  }
}

void tailJ(int goSpeed){
  Serial.println("J tail");
  servoTarget[0] = 110;
  servoTarget[1] = 0;
  servoTarget[2] = 110;
  servoTarget[3] = 0;
  servoTarget[4] = 90;
  servoTarget[5] = 0;
  servoTarget[6] = 70;
  servoTarget[7] = 0;
  goToTarget(goSpeed);
}

void tailReverseJ(int goSpeed){
  Serial.println("Tuck tail reversed J");
  //move to center to avoid hitting leggs
  servoTarget[0] = 0;
  servoTarget[1] = 0;
  servoTarget[2] = 0;
  servoTarget[3] = 0;
  servoTarget[4] = 0;
  servoTarget[5] = 0;
  servoTarget[6] = 0;
  servoTarget[7] = 0;
  goToTarget(goSpeed);
  
  servoTarget[0] = -110;
  servoTarget[1] = 0;
  servoTarget[2] = -80;
  servoTarget[3] = 0;
  servoTarget[4] = -50;
  servoTarget[5] = 0;
  servoTarget[6] = -30;
  servoTarget[7] = 0;
  goToTarget(goSpeed);
  keyPad();
  while (!Serial.available() && !keyPad() && !keyPadNewTask) {
  }
  //move to center to avoid hitting leggs
  servoTarget[0] = 0;
  servoTarget[1] = 0;
  servoTarget[2] = 0;
  servoTarget[3] = 0;
  servoTarget[4] = 0;
  servoTarget[5] = 0;
  servoTarget[6] = 0;
  servoTarget[7] = 0;
  goToTarget(goSpeed);
}

void printMenu(){
  Serial.println("Menu:");
  Serial.println("0 = Togle servos On/Off");
  Serial.println("1 = Center all servos -Caution!- return to known state");
  Serial.println("2 = ");
  Serial.println("3 = Tail down");
  Serial.println("4 = ");
  Serial.println("5 = J tail");
  Serial.println("6 = Tuck tail reversed J");
  Serial.println("7 = Wag tail low");
  Serial.println("8 = Wag tail high");
}

bool keyPad(){
    bool keyPressedReturn = false;
    char keypressed = kpd.getKey();
    if (keyPadTask != keypressed && keypressed != NO_KEY)
    {
      Serial.print("key pressed: ");
      keyPadTask = keypressed;
      Serial.println(keypressed);
      keyPressedReturn = true;
      keyPadNewTask = true;
    }
    return keyPressedReturn;
}

void loop() {
  if (Serial.available() > 0 || keyPad() || keyPadNewTask) {
    task = Serial.readString();

    if (task == "0\n" || keyPadTask == char('0')){
      if (active){
        pwm.sleep();
        digitalWrite(redLED, HIGH);
        digitalWrite(greenLED, LOW);
      }
      else {
        for (int i=0; i<=7; i++ ){
          servoTarget[i] = 0;
        }
        goToTarget(2);
        pwm.wakeup();
        digitalWrite(redLED, LOW);
        digitalWrite(greenLED, HIGH);
      }
      active = !active;
      Serial.print("Servos active set to: ");
      Serial.println(active);
      keyPadNewTask = false;
    }

    if (task == "1\n" || keyPadTask == char('1')){
      keyPadNewTask = false;
      cernterServos();
      printMenu();
    }
    if (task == "2\n" || keyPadTask == char('2')){
      Serial.println("Future function");
      keyPadNewTask = false;
      printMenu();
    }
    if (task == "3\n" || keyPadTask == char('3')){
      keyPadNewTask = false;
      tailDown(2);
      printMenu();
    }
    if (task == "4\n" || keyPadTask == char('4')){
      Serial.println("Future function");
      keyPadNewTask = false;
      printMenu();
    }  
    if (task == "5\n" || keyPadTask == char('5')){
      keyPadNewTask = false;
      tailJ(2);
      printMenu();
    }
    if (task == "6\n" || keyPadTask == char('6')){
      keyPadNewTask = false;
      tailReverseJ(5);
      printMenu();
    }
    if (task == "7\n" || keyPadTask == char('7')){
      keyPadNewTask = false;
      wagTailLow(1);
      printMenu();
    }
    if (task == "8\n" || keyPadTask == char('8')){
      keyPadNewTask = false;
      wagTailHigh(1);
      printMenu();
    }
  }
}
