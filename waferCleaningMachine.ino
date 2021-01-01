#include <Stepper.h>
#include <Vector.h> 
#include <ProcedureCuciMemory.h>
#include <UTFT.h>
#include <Key.h>
#include <Keypad.h>
#include <EEPROM.h>

unsigned long interval = 1000; 
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

const byte ROWS = 4; //Four Rows
const byte COLS = 4; //Four Cols

//Defince the keypads in matrix
char hexaKeys [ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte colPins[COLS] = {A7,A6,A5,A4}; //connect to the column pinouts of the keypad
byte rowPins[ROWS] = {A3,A2,A1,A0}; //connect to the row pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 
UTFT myGLCD(ILI9481,38,39,40,41);

extern uint8_t BigFont[];

Stage stage1 = Stage(1, 1, 1000, 25, 1);
Stage stage2 = Stage(2, 2, 1500, 25, 1);
Stage stage3 = Stage(3, 3, 2000, 25, 1);
Stage stage4 = Stage(4, 4, 2500, 25, 1);
Stage stage5 = Stage(5, 5, 2750, 25, 1);
Stage stage6 = Stage(6, 4, 2500, 25, 1);
Stage stageH202 = Stage(97, 97, 0, 100, 1); 
//Stage stage7 = Stage(7);
//Stage stage8 = Stage(8);
//Stage stage9 = Stage(9);
//Stage stage10 = Stage(10);
//Stage stage11 = Stage(11);
//Stage stage12 = Stage(12);
//Stage stage13 = Stage(13);
//Stage stage14 = Stage(14);
//Stage stage15 = Stage(15);
//Stage stage16 = Stage(16);
//Stage stage17 = Stage(17);
//Stage stage18 = Stage(18);
//Stage stage19 = Stage(19);
//Stage stage20 = Stage(20);
Stage stageAir = Stage(98, 98, 2000, 10, 1);
bool air = false;
bool n2 = false;
Stage stageNitrogen = Stage(99, 99, 3000, 10, 1);

//Stage stageRCA[] = {stage1,stage2,stage3,stage4,stage5, stage6, stage7, stage8,
//stage9, stage10, stage11, stage12, stage13, stage14, stage15, stage16, stage17,
//stage18, stage19, stage20};

Stage stageRCA[] = {stage1,stage2,stage3,stage4,stage5, stage6};
Stage arrStageH202 [] = {stageH202};
//Stage stageRCA2[] = {stage1,stage2,stage3,stage4,stage5, stage6, stage7, stage8,
//stage9, stage10, stage11, stage12, stage13, stage14, stage15};
  
Stage sdStage1 = Stage(1);

Stage standardStage[] = {sdStage1};

Procedure procedureRCA = Procedure(1, 6, stageRCA);
Procedure procedureH202 = Procedure(99, 1, arrStageH202);
//Procedure procedureRCA2 = Procedure(2, 20, stageRCA);
//Procedure procedureRCA3 = Procedure(3, 20, stageRCA);
//Procedure procedureRCA4 = Procedure(4, 20, stageRCA);

Procedure currentProcedure;
Procedure tempProcedure;

vector<Procedure> allProcedure;
vector<int> savedProcedure;

vector<Stage> allStage;

int numberProcedure;

enum DisplayState{
  MENU_A,
  MENU_B,
  MENU_C,
  MENU_D,
  MENU_E,
  MENU_F,
  MENU_G,
  MENU_MAX
};

DisplayState displayState = MENU_A;
DisplayState lastDisplayState;
byte pinValveAir = A11; 
byte pinNitrogen = A12;
byte pinValve[]={12,11,10,8,14,15,16,17,18,19,20,21,A15,A14,A13};
//byte pinValve[]={12,11,10,8,14,19,18,17,16,15,20,21,A15,A14,A13};
//byte pinValve[]={14,8,10,11,12,19,18,17,16,15,20,21,A15,A14,A13};

vector <int> valveStage;

//int pinValve[]={8,7,5,4,2,10,11,12,14,15,17,18,19,20,21};
//Steper 2 4
//Sensor RPM 3
//Sensor Jarak 5 7
//Motor 6
//Servo 9
//PinValve 
//8 10 11 12 14 
//15 16 17 18 19 
//20 21 A13 A14 A15
//Keypad A0-A7
//LED A8-A10
//Pin Nitronen A11
//pin pompa A12

byte pinLED[] = {A8, A9, A10};

int j=0;
int eeAddress = 0;
int arraySaveProcedure[500];

int numberProcedureProcessed = 0;
int i;
int currentStage=1;
bool pencucianSelesai = false;
int timeLeftTotal = 0;

int count1=0;
int motor_speed = 0;
int actual_speed = 0;
int pwm_now = 0;
int desired_speed = 0;
int error0, error1, error2, lasterror;
float result;
float kp, kd, ki;
int result_pwm;

const byte interruptPin = 3;
bool stepMove = false;
bool stepMoveFirst = false;

// twelve servo objects can be created on most boards
const int stepsPerRevolution = 800;  // change this to fit the number of steps per revolution
// for your motor
// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 2, 4, 9, 13);

// defines pins numbers for range sensor
const int trigPin = 5;
const int echoPin = 7;

// defines variables for range sensor
long duration;
int distanceStepper;

int theStep = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  // Set trig and echo pin as output and input
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  randomSeed(analogRead(0));
  
  // set the speed at 60 rpm
  timeLeftTotal = 0;
  count1=0;
  actual_speed = 0;
  pwm_now = 0;
  desired_speed = 0;
  myStepper.setSpeed(60);
  set_timer1(); //set timer 1 interrupt at 1Hz
  loadProcedure();
  if (allProcedure.empty()){
    allProcedure.push_back(procedureRCA);
    allProcedure.push_back(procedureH202);
  } 
  Serial.println(procedureRCA.getNumberStage());
//  saveProcedure();
//  Serial.println("A");
  pinMode(pinValve[0], OUTPUT);
  digitalWrite(pinValve[0], HIGH);
  pinMode(pinValve[1], OUTPUT);
  digitalWrite(pinValve[1], HIGH);
  pinMode(pinValve[2], OUTPUT);
  digitalWrite(pinValve[2], HIGH);
  pinMode(pinValve[3], OUTPUT);
  digitalWrite(pinValve[3], HIGH);
  pinMode(pinValve[4], OUTPUT);
  digitalWrite(pinValve[4], HIGH);
  pinMode(pinValve[5], OUTPUT);
  digitalWrite(pinValve[5], HIGH);
  pinMode(pinValve[6], OUTPUT);
  digitalWrite(pinValve[6], HIGH);
  pinMode(pinValve[7], OUTPUT);
  digitalWrite(pinValve[7], HIGH);
  pinMode(pinValve[8], OUTPUT);
  digitalWrite(pinValve[8], HIGH);
  pinMode(pinValve[9], OUTPUT);
  digitalWrite(pinValve[9], HIGH);
  pinMode(pinValve[10], OUTPUT);
  digitalWrite(pinValve[10], HIGH);
  pinMode(pinValve[11], OUTPUT);
  digitalWrite(pinValve[11], HIGH);
  pinMode(pinValve[12], OUTPUT);
  digitalWrite(pinValve[12], HIGH);
  pinMode(pinValve[13], OUTPUT);
  digitalWrite(pinValve[13], HIGH);
  pinMode(pinValve[14], OUTPUT);
  digitalWrite(pinValve[14], HIGH);
  
  //pinLED
  pinMode(pinLED[0], OUTPUT);
  digitalWrite(pinLED[0], HIGH);
  pinMode(pinLED[1], OUTPUT);
  digitalWrite(pinLED[1], LOW);
  pinMode(pinLED[2], OUTPUT);
  digitalWrite(pinLED[2], HIGH);

  //pin Valve air dan nitrogen
  pinMode(pinValveAir, OUTPUT);
  digitalWrite(pinValveAir, HIGH);
  pinMode(pinNitrogen, OUTPUT);
  digitalWrite(pinNitrogen, HIGH);
  
  myGLCD.InitLCD();
  myGLCD.setFont(BigFont);
  myGLCD.clrScr();

  desired_speed = 0;

  air = false;
  digitalWrite(pinValveAir, HIGH);
  n2 = false;
  digitalWrite(pinNitrogen, HIGH);
  printStartScreen();
  
  myGLCD.fillScr(40,85,154);
  myGLCD.setBackColor(40,85,154);
  myGLCD.setColor(255,255,255);
  myGLCD.print("Apakah anda sudah", CENTER, 140);
  myGLCD.print("meletakkan wafer?", CENTER, 160);
  myGLCD.print("Jika ya tekan D", CENTER, 180);
  myGLCD.print("Jika tidak tekan B", CENTER, 200);
  char key = NO_KEY;
  while (key == NO_KEY){
    key = keypad.getKey();
  }
  if (key == 'D'){
    
  } else {
    myGLCD.fillScr(40,85,154);
    myGLCD.setColor(255,255,255);
    myGLCD.print("Mohon menunggu", CENTER, 140);
    myGLCD.print("Stepper sedang", CENTER, 160);
    myGLCD.print("menuju posisi awal", CENTER, 180);
    //17500
    
//    moveStepper_pos(14500);    
  }
  
  chooseProcedureMenu();
  currentProcedure = allProcedure[numberProcedureProcessed];
  if ((pencucianSelesai==true)&&(currentStage > currentProcedure.getNumberStage())){
    currentStage = 1;
    pencucianSelesai = false;
    stageNitrogen.setResetTime();
    for (i=0; i<sizeof(currentProcedure.getStage()); i++){
       currentProcedure.getStage()[i].setResetTime();
    }
  }
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), Rpm_Sensor, RISING);
  //Set interrupt timer for range sensor reading
  
  //Set distance monitoring
  previousMillis = millis();
  
  setValveStage(currentProcedure.getStage()[0], &valveStage);  
  
  myGLCD.fillScr(40,85,154);
  myGLCD.setColor(255,255,255);
  myGLCD.print("Letakkan sample wafer", CENTER, 150);
  myGLCD.print("yang ingin dicuci", CENTER, 170);
  myGLCD.print("Lalu tekan enter", CENTER, 200);
  
  key = NO_KEY;
  while (key!='D'){
    key = keypad.getKey();
  }
  myGLCD.fillScr(40,85,154);
  myGLCD.setColor(255,255,255);
  myGLCD.print("Mohon menunggu", CENTER, 140);
  myGLCD.print("Stepper sedang menuju", CENTER, 160);
  myGLCD.print("posisi tahapan pertama", CENTER, 180);
//  moveStepper(0);
  myGLCD.fillScr(40,85,154);
  for (int i=0; i<currentProcedure.getNumberStage(); i++){
    timeLeftTotal += currentProcedure.getStage()[i].getTime();
    timeLeftTotal += stageAir.getTime();
  }
  timeLeftTotal += stageNitrogen.getTime();
  digitalWrite(pinLED[1], HIGH);
  delay(2000);
  digitalWrite(pinLED[2], LOW);
  delay(2000);
}

void loop() {
  if ((!air) && (!n2)){
    desired_speed = currentProcedure.getStage()[currentStage-1].getRPM();
  } else if (air){
    desired_speed = stageAir.getRPM();
  } else if (n2){
    desired_speed = stageNitrogen.getRPM();
  }  
  analogWrite(6,pwm_now);
  currentMillis = millis();
  
  //Stage 1
  if ((unsigned long)(currentMillis-previousMillis)>=interval){
    if (air){
//      myGLCD.setColor(0,0,255);
//      myGLCD.fillRoundRect(0, 40, 480, 60);
      myGLCD.setColor(255,255,255);
      myGLCD.print(stageName(stageAir.getName()), CENTER, 40);
      myGLCD.print("Fluid", LEFT, 100);
      myGLCD.print(": " + fluidName(stageAir.getFluid()), 200, 100, 0);
      if ((stageAir.getTime() == 9) || (stageAir.getTime() == 99) || (stageAir.getTime() == 999)){
        myGLCD.setColor(40,85,154);    
        myGLCD.fillRoundRect(0,120,480,140);
        myGLCD.setColor(255,255,255);
        myGLCD.print("Time", LEFT, 120);
        myGLCD.print(": " +(String)(stageAir.getTime()), 200, 120, 0);
      } else {
        myGLCD.print("Time", LEFT, 120);
        myGLCD.print(": " +(String)(stageAir.getTime()), 200, 120, 0);
      }
      myGLCD.print("RPM", LEFT, 140);
      myGLCD.setColor(40,85,154);    
      myGLCD.fillRoundRect(200,140,480,160);
      myGLCD.setColor(255,255,255);
      myGLCD.print(": " +String(actual_speed), 200, 140, 0);
      myGLCD.print("Size", LEFT, 160);
      myGLCD.print(": " +sizeName(stageAir.getSize()), 200, 160, 0);
      if ((timeLeftTotal == 99) || (timeLeftTotal == 9) || (timeLeftTotal == 999)){
        myGLCD.setColor(40,85,154);    
        myGLCD.fillRoundRect(0,180,480,200);
        myGLCD.setColor(255,255,255);
      }
      myGLCD.print("Time Left", LEFT, 180);
      myGLCD.print(": " +String(timeLeftTotal), 200, 180, 0);
      timeLeftTotal -=1;
      stageAir.setTime(stageAir.getTime()-1); 
      previousMillis = millis();
    } else if (n2){
      myGLCD.setColor(255,255,255);
      myGLCD.print(stageName(stageNitrogen.getName()), CENTER, 40);
      myGLCD.print("Fluid", LEFT, 100);
      myGLCD.print(": " +fluidName(stageNitrogen.getFluid()), 200, 100, 0);
      if ((stageNitrogen.getTime() == 9) || (stageNitrogen.getTime() == 99) || (stageNitrogen.getTime() == 999)){
        myGLCD.setColor(40,85,154);    
        myGLCD.fillRoundRect(0,120,480,140);
        myGLCD.setColor(255,255,255);
        myGLCD.print("Time", LEFT, 120);
        myGLCD.print(": " +(String)(stageNitrogen.getTime()), 200, 120, 0);
      } else {
        myGLCD.print("Time", LEFT, 120);
        myGLCD.print(": " +(String)(stageNitrogen.getTime()), 200, 120, 0);
      }
      myGLCD.print("RPM", LEFT, 140);
      myGLCD.setColor(40,85,154);    
      myGLCD.fillRoundRect(200,140,480,160);
      myGLCD.setColor(255,255,255);
      myGLCD.print(": " +String(actual_speed), 200, 140, 0);
      myGLCD.print("Size", LEFT, 160);
      myGLCD.print(": " +sizeName(stageNitrogen.getSize()), 200, 160, 0);
      stageNitrogen.setTime(stageNitrogen.getTime()-1); 
      if ((timeLeftTotal == 99) || (timeLeftTotal == 9) || (timeLeftTotal == 999)){
        myGLCD.setColor(40,85,154);    
        myGLCD.fillRoundRect(0,180,480,200);
        myGLCD.setColor(255,255,255);
      }
      myGLCD.print("Time Left", LEFT, 180);
      myGLCD.print(": " +String(timeLeftTotal), 200, 180, 0);
      timeLeftTotal -=1;
      previousMillis = millis();   
    }else 
    if ((!air) && !(n2)){
//      myGLCD.setColor(0,0,255);
//      myGLCD.fillRoundRect(0, 40, 480, 60);
      myGLCD.setColor(255,255,255);
      myGLCD.print(stageName(currentProcedure.getStage()[currentStage-1].getName()), CENTER, 40);
      myGLCD.print("Fluid" , LEFT, 100);
      myGLCD.print(": " +fluidName(currentProcedure.getStage()[currentStage-1].getFluid()), 200, 100, 0);
      if ((currentProcedure.getStage()[currentStage-1].getTime() == 9)||(currentProcedure.getStage()[currentStage-1].getTime() == 99) ||(currentProcedure.getStage()[currentStage-1].getTime() == 999)){
        myGLCD.setColor(40,85,154);    
        myGLCD.fillRoundRect(0,120,480,140);
        myGLCD.setColor(255,255,255);
        myGLCD.print("Time", LEFT, 120);
        myGLCD.print(": " +(String)(currentProcedure.getStage()[currentStage-1].getTime()), 200, 120, 0);
      } else {
        myGLCD.print("Time", LEFT, 120);
        myGLCD.print(": " +(String)(currentProcedure.getStage()[currentStage-1].getTime()), 200, 120, 0);
      } 
//      if (currentProcedure.getStage()[currentStage-1].getTime() %10 ==0){
//        if (stepMoveFirst){
//          if (stepMove){
//            Serial.println("kanan");
//            myStepper.step(800);
//            myStepper.step(800);
//            myStepper.step(800);
//            stepMove = false;
//          } else {
//            Serial.println("kiri");
//            myStepper.step(-800);          
//            myStepper.step(-800);          
//            myStepper.step(-800);
//            stepMove = true;
//          }  
//        } else {
//            Serial.println("Awal");
//            myStepper.step(800);
//            myStepper.step(800);
//            stepMoveFirst = true;
//        }
//        
//      }
        
      myGLCD.print("RPM", LEFT, 140);
      myGLCD.setColor(40,85,154);    
      myGLCD.fillRoundRect(200,140,480,160);
      myGLCD.setColor(255,255,255);
      myGLCD.print(": " +String(actual_speed), 200, 140, 0);
      myGLCD.print("Size", LEFT, 160);
      myGLCD.print(": " +sizeName(procedureRCA.getStage()[currentStage-1].getSize()), 200, 160, 0);
      myGLCD.print("Stage Left", LEFT, 180);
      myGLCD.print(": " +String(procedureRCA.getNumberStage()-currentStage), 200, 180, 0);
      currentProcedure.getStage()[currentStage-1].setTime(currentProcedure.getStage()[currentStage-1].getTime()-1); 
      if ((timeLeftTotal == 99) || (timeLeftTotal == 9) || (timeLeftTotal == 999)){
        myGLCD.setColor(40,85,154);    
        myGLCD.fillRoundRect(0,200,480,220);
        myGLCD.setColor(255,255,255);
      }
      myGLCD.print("Time Left", LEFT, 200);
      myGLCD.print(": " +String(timeLeftTotal), 200, 200, 0);
      timeLeftTotal -=1;
      previousMillis = millis();
    } 
  }
  
  if (air){ 
    if (stageAir.getTime()==0){
      myGLCD.setColor(0,0,255);
      myGLCD.fillRoundRect(0, 40, 480, 60);
      air = false;
      stageAir.setResetTime();
      myGLCD.clrScr();
      myGLCD.fillScr(40,85,154);
      digitalWrite(pinValveAir, HIGH);
      i = currentStage-2;
      digitalWrite(pinValve[valveStage[2]-1], HIGH);
    } else {
      digitalWrite(pinValveAir, LOW);
      i = currentStage-2;
      digitalWrite(pinValve[valveStage[2]-1], LOW);
    }
  } else if ((!air) && (!n2)){
    if (i == 1){
      delay(100);
      digitalWrite(pinValve[valveStage[2]], LOW);
      delay(100);
      digitalWrite(pinValve[valveStage[0]], LOW);
      delay(100);
    } else {
      i = currentStage-1;
      digitalWrite(pinValve[valveStage[0]], LOW);
      digitalWrite(pinValve[valveStage[2]], LOW);
    }
  }

  if (n2){
    if (stageNitrogen.getTime() == 0){
      myGLCD.setColor(0,0,255);
      myGLCD.fillRoundRect(0, 40, 480, 60);
      digitalWrite(pinNitrogen, HIGH);
      n2 = false;
    } else {
      digitalWrite(pinNitrogen, LOW);
    }
  }

  
  if ((currentStage <= currentProcedure.getNumberStage()) && (!air)){
    if (currentProcedure.getStage()[currentStage-1].getTime()>2){
      i = currentStage-1;
      digitalWrite(pinValve[valveStage[1]], LOW);
    } else if (currentProcedure.getStage()[currentStage-1].getTime()==2){
        i = currentStage-1;
        digitalWrite(pinValve[valveStage[1]], HIGH);
        digitalWrite(pinValveAir, LOW);
    } else if (currentProcedure.getStage()[currentStage-1].getTime()==0){
        myGLCD.fillScr(40,85,154);
        myGLCD.setColor(255,255,255);
        i = currentStage-1;
        for (j=0; j<2;j++){
          digitalWrite(pinValve[valveStage[j]], HIGH);
        }
        currentStage=currentStage + 1;
        if (currentStage<=currentProcedure.getNumberStage()){
          myGLCD.fillScr(40,85,154);
          myGLCD.print("Mohon menunggu", CENTER, 140);
          myGLCD.print("Stepper sedang menuju", CENTER, 160);
          myGLCD.print("posisi tahapan berikutnya", CENTER, 180);
//          moveStepper(currentStage-1);
          myGLCD.fillScr(40,85,154);
        }
        valveStage.clear();
        if (currentStage <= currentProcedure.getNumberStage()){
          setValveStage(currentProcedure.getStage()[currentStage-1], &valveStage);   
        }
        air = true;
        stepMoveFirst = false;
    }
  } else{
    if (!air){
      if (!n2){
        if (stageNitrogen.getTime() == 0){ 
          myGLCD.fillScr(40,85,154);
          myGLCD.setColor(255,255,255);
          myGLCD.print("Pencucian telah selesai", CENTER, 140);
          myGLCD.print("Silakan ambil sample", CENTER, 160);
          myGLCD.print("wafer anda", CENTER, 180);
          myGLCD.print("Press Any Key", CENTER, 220);
          
          digitalWrite(pinLED[2], HIGH);
          delay(100);
          digitalWrite(pinLED[0], LOW);
          delay(100);
          pencucianSelesai = true;
          desired_speed = 0;
          pwm_now = 0;
          analogWrite(6,pwm_now);
          char key = keypad.getKey();
          while(key == NO_KEY){
            key = keypad.getKey();
            desired_speed = 0;
          }
          setup();
        } else if (stageNitrogen.getTime() > 0){
          myGLCD.fillScr(40,85,154);
          myGLCD.setColor(255,255,255);
          myGLCD.print("Mohon menunggu", CENTER, 140);
          myGLCD.print("Stepper sedang menuju", CENTER, 160);
          myGLCD.print("posisi akhir", CENTER, 180);
//          moveStepper_pos(18000);
          myGLCD.fillScr(40,85,154);
          n2 = true;
        }
      }
    }
  } 
}

//////////////////////////////////////////////////////////
////////////////////END OF VOID LOOP//////////////////////
//////////////////////////////////////////////////////////

void setValveStage(Stage currentStage, vector<int> *valveStage){
//  valveStage->clear();
  if ((fluidName(currentStage.getFluid()) == "SPM") || (fluidName(currentStage.getFluid()) == "H202")){
    valveStage->push_back(0);
    valveStage->push_back(5);
    valveStage->push_back(10);
  } else if (fluidName(currentStage.getFluid()) == "Acetone"){
    valveStage->push_back(1);
    valveStage->push_back(6);
    valveStage->push_back(11);
  } else if (fluidName(currentStage.getFluid()) == "APM"){
    valveStage->push_back(2);
    valveStage->push_back(7);
    valveStage->push_back(12);
  } else if (fluidName(currentStage.getFluid()) == "DHF"){
    valveStage->push_back(3);
    valveStage->push_back(8);
    valveStage->push_back(13);
  } else if (fluidName(currentStage.getFluid()) == "HPM"){
    valveStage->push_back(4);
    valveStage->push_back(9);
    valveStage->push_back(14);
  } 
}

void printStartScreen(){
  //Red rectangle
  myGLCD.fillScr(40,85,154);
  myGLCD.setColor(255,0,0);
  myGLCD.fillRoundRect(100, 70, 380, 169);
  
  //Text
  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(255, 0, 0);
  myGLCD.print("AWACLES", CENTER, 93);
  myGLCD.print("Automatic Wafer", CENTER, 120);
  myGLCD.print("Cleaning System", CENTER, 140);
  
  myGLCD.setBackColor(40,85,154);
  myGLCD.print("TA181901017", CENTER, 280);

  delay(2000);
}

String procedureName(int numberProcedure){
  if (numberProcedure == 0){
    return "New Procedure";
  } else if (numberProcedure == 1){
    return "Procedure RCA";
  } else if (numberProcedure == 99){
    return "Procedure H202";
  }
  else {
    return ("Procedure "+(String)(numberProcedure));
  }
}

String stageName(int numberStage){
  if (numberStage == 99){
    return "Stage Nitrogen";
  } else if (numberStage == 98){
    return "Stage Air";
  } else if (numberStage == 97){
    return "Stage H202";
  } else {
    return ("Stage " + (String)(numberStage));
  }
}

String fluidName(int numberFluid){
  if (numberFluid == 1){
    return "SPM";
  } else if (numberFluid == 2){
    return "Acetone";
  } else if (numberFluid == 3){
    return "APM";
  } else if (numberFluid == 4){
    return "DHF";
  } else if (numberFluid == 5){
    return "HPM";
  } else if (numberFluid == 97){
    return "H202";
  } else if (numberFluid == 98){
    return "Air";
  } else if (numberFluid == 99){
    return "Nitrogen";
  } else {
    return "";
  }
}

String sizeName(int numberSize){
  if (numberSize == 1){
    return "3 x 3";
  } else if (numberSize == 2){
    return "2 x 2";
  } else {
    return "";
  }
}

void moveStepper(int currentStage){
  String stageName = fluidName(currentProcedure.getStage()[currentStage].getFluid());
  int target = 0;
  if ((stageName == "SPM")||(stageName == "H202")){
    target = 4500;
  } else if (stageName=="Acetone"){
    target = 8000;
  } else if (stageName=="APM"){
    target = 10000;
  } else if (stageName=="DHF"){
    target = 12000;
  } else if (stageName=="HPM"){
    target = 15000;
  }
//  Serial.print("Target :");
//  Serial.println(target);
  if ((distanceStepper>target-100) && (distanceStepper<target+100)){}
  else if (distanceStepper < target){
    while (distanceStepper < target-100){
      theStep = stepsPerRevolution;
      myStepper.step(theStep);
//      Serial.println("atas");
//      Serial.println(target);
//      Serial.println(stageName);
//      Serial.println(distanceStepper - target);
      Serial.println(distanceStepper);
    }
    theStep = 0;
    myStepper.step(theStep);
  } else if (distanceStepper > target){
    while (distanceStepper > target+100){
      theStep = -stepsPerRevolution;
      myStepper.step(theStep);
      
      Serial.println(distanceStepper);

      //5100
      //8200
      //10000
      //12000
      //15000

      
//      Serial.println("bawah");
//      Serial.println(target);
//      Serial.println(stageName);
//      Serial.println(distanceStepper - target + 500);
    }
    theStep = 0;
    myStepper.step(theStep);
  } else {
    theStep = 0;
    myStepper.step(theStep);
  }
}

void moveStepper_pos(int target){
//  Serial.print("Target :");
//  Serial.println(target);
  if ((distanceStepper>target-100) && (distanceStepper<target+100)){}
  else if (distanceStepper < target){
    while (distanceStepper < target-100){
      theStep = stepsPerRevolution;
      myStepper.step(theStep);
      
      Serial.println(distanceStepper);
    }
    theStep = 0;
    myStepper.step(theStep);
  } else if (distanceStepper > target){
    while (distanceStepper > target+100){
      theStep = -stepsPerRevolution;
      myStepper.step(theStep);
      
      Serial.println(distanceStepper);
    }
    theStep = 0;
    myStepper.step(theStep);
  } else {
    theStep = 0;
    myStepper.step(theStep);
  }
}

void chooseProcedureMenu(){
  displayState = MENU_A;
  lastDisplayState = allProcedure.size();
  int menumax = lastDisplayState+1;
//  myGLCD.fillScr(40,85,154);
//  myGLCD.setColor(255,255,255);
//  myGLCD.setBackColor(40,85,154);
//  myGLCD.print("Wafer Cleaning", CENTER, 40);
//  myGLCD.print("Choose Procedure :", LEFT, 100);
//  int piksel = 120;
//  for (int i=0; i<allProcedure.size();i++){
//    if (i == displayState){
//      myGLCD.setBackColor(255,0,0);
//      myGLCD.print(procedureName(allProcedure[i].getName()), LEFT, 120+(i*20));
//      myGLCD.setBackColor(40,85,154);
//    } else {
//      myGLCD.setBackColor(40,85,154);
//      myGLCD.print(procedureName(allProcedure[i].getName()), LEFT, 120+(i*20));
//    }
//  }
//  myGLCD.print("New Procedure", LEFT, 120+((allProcedure.size()+1)*20));
  
  char key = keypad.getKey();
  while (key!='D'){
    if(key=='8'){
      char currentState = static_cast<int>(displayState);
      displayState = static_cast<DisplayState>(++currentState % menumax);
    } else if (key == '2'){
      char currentState = static_cast<int>(displayState);
      displayState = static_cast<DisplayState>((currentState-1) < 0? static_cast<int>(menumax)-1 : --currentState);
    } 
    if (displayState != lastDisplayState){
      myGLCD.fillScr(40,85,154);
      myGLCD.setColor(255,255,255);
      myGLCD.setBackColor(40,85,154);
      myGLCD.print("Wafer Cleaning", CENTER, 40);
      myGLCD.print("Choose Procedure :", LEFT, 100);
      int piksel = 120;
      for (int i=0; i<allProcedure.size();i++){
        if (i == displayState){
          myGLCD.setBackColor(255,0,0);
          myGLCD.print(procedureName(allProcedure[i].getName()), LEFT, 120+(i*20));
          myGLCD.setBackColor(40,85,154);
        } else {
          myGLCD.setBackColor(40,85,154);
          myGLCD.print(procedureName(allProcedure[i].getName()), LEFT, 120+(i*20));
        }
      }
      if (displayState == (menumax-1)){
        myGLCD.setColor(255,255,255);
        myGLCD.setBackColor(255,0,0);
        myGLCD.print("New Procedure", LEFT, 120+((allProcedure.size()+1)*20));
      } else {
        myGLCD.setColor(255,255,255);
        myGLCD.setBackColor(40,85,154);
        myGLCD.print("New Procedure", LEFT, 120+((allProcedure.size()+1)*20));
      }
      lastDisplayState = displayState;
    }
    if (key == 'B'){
      myGLCD.fillScr(40,85,154);
      myGLCD.print("You cannot back from this page", CENTER, 160);
      delay(1500);
      chooseProcedureMenu();
    }
    key = NO_KEY;
    while (key==NO_KEY){
      key = keypad.getKey();
    }
  } 
  if (key == 'D'){
    if (displayState == MENU_A){
      choiceProcedure(&procedureRCA);
    } else if (displayState == allProcedure.size()){
      if (allProcedure.size()==4){
        myGLCD.fillScr(40,85,154);
        myGLCD.setColor(255,255,255);
        myGLCD.setBackColor(40,85,154);
        myGLCD.print("Prosedur sudah mencapai", CENTER, 140);
        myGLCD.print("batas maksimum", CENTER, 160);
        myGLCD.print("(Maksimum 4 Prosedur)", CENTER, 180);
        myGLCD.setBackColor(255,0,0);
        delay(3000);
        chooseProcedureMenu();
      }
      Procedure newProcedure = Procedure(allProcedure.size()+1,1,standardStage);
      editProcedure(&newProcedure);
      allProcedure.push_back(newProcedure);
      saveProcedure();
      chooseProcedureMenu();
    } else {
      giveNumber(&numberProcedureProcessed, displayState);
      choiceProcedure(&allProcedure[displayState]);
    }
  }
}

void choiceProcedure(Procedure *newProcedure){
  displayState = MENU_A;
  lastDisplayState = MENU_A;
  int menumax = MENU_B+1;
  
  myGLCD.fillScr(40,85,154);
  myGLCD.setColor(255,255,255);
  myGLCD.print(procedureName(newProcedure->getName()), CENTER, 20);
  myGLCD.print("Start Prosedur?", LEFT, 100);
  myGLCD.setBackColor(255,0,0);
  myGLCD.print("Start", LEFT, 120);
  myGLCD.setBackColor(40,85,154);
  myGLCD.print("Edit", LEFT, 140);
  
  char key = keypad.getKey();
  while (key==NO_KEY){
    key = keypad.getKey();
  }

  while (key!='D'){
    if(key=='8'){
      if (displayState==MENU_A) {
        displayState=MENU_B;
      } else 
        displayState=MENU_A;
    } else if (key == '2'){
      if (displayState==MENU_B){
        displayState=MENU_A;  
      } else
        displayState=MENU_B;
    } 
    if (displayState != lastDisplayState){
      switch(displayState){
        case MENU_A:
          myGLCD.fillScr(40,85,154);
          myGLCD.setBackColor(40,85,154);
          myGLCD.setColor(255,255,255);
          myGLCD.print(procedureName(newProcedure->getName()), CENTER, 20);
          myGLCD.print("Start Prosedur?", LEFT, 100);
          myGLCD.setBackColor(255,0,0);
          myGLCD.print("Start", LEFT, 120);
          myGLCD.setBackColor(40,85,154);
          myGLCD.print("Edit", LEFT, 140);
          break;
        case MENU_B:
          myGLCD.fillScr(40,85,154);
          myGLCD.setBackColor(40,85,154);
          myGLCD.setColor(255,255,255);
          myGLCD.print(procedureName(newProcedure->getName()), CENTER, 20);
          myGLCD.print("Start Prosedur?", LEFT, 100);
          myGLCD.print("Start", LEFT, 120);
          myGLCD.setBackColor(255,0,0);
          myGLCD.print("Edit", LEFT, 140);
          break;
      }
      lastDisplayState = displayState;
    }
    if (key=='B'){
      chooseProcedureMenu();
    }
    key = NO_KEY;
    while (key==NO_KEY){
      key = keypad.getKey();
    }
    
  } 
  if (key == 'D'){
    switch (displayState){
      case MENU_A:
      {
        break;  
      }
      case MENU_B:
      {
        editProcedure(newProcedure);
        saveProcedure();
        chooseProcedureMenu();
      }
      default:
        chooseProcedureMenu();
    }
  }
}

void giveNumber(int *numberProcedureProcessed, int displayState){
  *numberProcedureProcessed = displayState;
}

void editProcedure(Procedure *newProcedure){
  displayState = MENU_A;
  lastDisplayState = newProcedure->getNumberStage()+5;
  int menumax = lastDisplayState+1;
  int displayPage = 0;
  int lastDisplayPage = 0;
  int maxPage = newProcedure->getNumberStage()/5;
  int stageChosen = 0;
  vector<String> cetak;
  vector<String> cetakStage; 
  cetak.push_back("Stage Air");
  cetak.push_back("Stage Nitrogen");
  cetak.push_back("Add Stage");
  cetak.push_back("Edit Procedure Name");
  cetak.push_back("Delete Procedure");
 
  char key = keypad.getKey();
  
  while (key!='D'){
//    Serial.print(key);
    if(key=='8'){
      char currentState = static_cast<int>(displayState);
      displayState = static_cast<DisplayState>(++currentState % menumax);
      stageChosen++;
    } else if (key == '2'){
      char currentState = static_cast<int>(displayState);
      displayState = static_cast<DisplayState>((currentState-1) < 0? static_cast<int>(menumax-1) : --currentState);
      stageChosen--;
    } 
    Serial.print("Max page :");
    Serial.println(maxPage);
    if (maxPage>0){  
      if ((key=='6') && (displayPage<maxPage)){
        displayPage++;
        stageChosen+=5;      
      } else if ((key=='4') && (displayPage>0)){
        displayPage--;
        stageChosen-=5;
      }
      cetakStage.clear();
      for (int k=5*displayPage; k<(5*displayPage)+5; k++){
        if (k<newProcedure->getNumberStage()){
          cetakStage.push_back(stageName(newProcedure->getStage()[k].getName()));
        }
      }
    } else {
      cetakStage.clear();
      for (int k=0; k<newProcedure->getNumberStage(); k++){
        cetakStage.push_back(stageName(newProcedure->getStage()[k].getName()));
      }
    }
    if ((displayState != lastDisplayState) || (displayPage!=lastDisplayPage)){
      myGLCD.fillScr(40,85,154);
      myGLCD.setColor(255,255,255);
      myGLCD.setBackColor(40,85,154);
      myGLCD.print("Edit" + procedureName(newProcedure->getName()), CENTER, 40);
      int piksel = 100;
      for (int i=0; i<cetakStage.size(); i++){
        if (i == displayState){
          myGLCD.setBackColor(255,0,0);
          myGLCD.print(cetakStage[i], LEFT, piksel+(i*20));
          myGLCD.setBackColor(40,85,154);
        } else {
          myGLCD.setBackColor(40,85,154);
          myGLCD.print(cetakStage[i], LEFT, piksel+(i*20));
        }
      }
      for (int j=cetakStage.size(); j<(cetak.size()+cetakStage.size()); j++){
        if (j == displayState){
          myGLCD.setBackColor(255,0,0);
          myGLCD.print(cetak[j-cetakStage.size()], LEFT, piksel+(j*20));
          myGLCD.setBackColor(40,85,154);
        } else {
          myGLCD.setBackColor(40,85,154);
          myGLCD.print(cetak[j-cetakStage.size()], LEFT, piksel+(j*20));
        }
      }
      if (displayState == (menumax-1)){
        myGLCD.setColor(255,255,255);
        myGLCD.setBackColor(255,0,0);
        myGLCD.print("OK", LEFT, 300);
        myGLCD.setBackColor(40,85,154);
      } else {
        myGLCD.setColor(255,255,255);
        myGLCD.setBackColor(40,85,154);
        myGLCD.print("OK", LEFT, 300);
      }
      lastDisplayState = displayState;
      lastDisplayPage = displayPage;
    }
    if (key=='B'){
      chooseProcedureMenu();
    }
    key = NO_KEY;
    while (key==NO_KEY){
      key = keypad.getKey();
    }
  }
//  Serial.println(displayState);
  if (key == 'D'){
    if (displayState == menumax-1){
         return ;
    } else if (displayState == menumax-2){
      if (newProcedure->getName() == 1){
          myGLCD.fillScr(40,85,154);
          myGLCD.setColor(255,255,255);
          myGLCD.print("RCA tidak bisa dihapus", CENTER, 200);
          editProcedure(newProcedure);
      } else {
        for (int i=1; i<allProcedure.size(); i++){
          if (newProcedure->getName() == allProcedure[i].getName()){
            allProcedure.erase(allProcedure.begin() +i);
          }
        }
        chooseProcedureMenu();
      }
    } else if (displayState == menumax-3){
      renameProcedure(newProcedure);
    } else if (displayState == menumax-4){
      addStage(newProcedure);
    } else if (displayState == menumax-5){
      editNonStage(newProcedure, &stageNitrogen);
      editProcedure(newProcedure);
    } else if (displayState == menumax-6){
      editNonStage(newProcedure, &stageAir);
      editProcedure(newProcedure);
    } else {
      editStage(newProcedure, &newProcedure->getStage()[stageChosen]);
      editProcedure(newProcedure);
    }
  }
}

void addStage(Procedure *newProcedure){
  allStage.clear();
  for (int i=0; i<newProcedure->getNumberStage(); i++){
    allStage.push_back(newProcedure->getStage()[i]);
  }
  Stage newStage = Stage(newProcedure->getNumberStage()+1);
  editStage(newProcedure, &newStage);
  allStage.push_back(newStage);
  Stage tempStage[allStage.size()];
  for (int i=0; i<allStage.size(); i++){
    tempStage[i] = allStage[i];
  }
  Procedure changedProcedure = Procedure(newProcedure->getName(), allStage.size(), tempStage);
  *newProcedure = changedProcedure;
  editProcedure(newProcedure);
}

void renameProcedure(Procedure *newProcedure){
  myGLCD.fillScr(40,85,154);
  myGLCD.setColor(255,255,255);
  myGLCD.setBackColor(40,85,154);
  myGLCD.print("Edit" + procedureName(newProcedure->getName()), CENTER, 40);
  String newName ="";
  myGLCD.print("Masukkan nama prosedur yang", LEFT, 100);
  myGLCD.print("baru (dalam bentuk angka):", LEFT, 120);
  
  char key = NO_KEY;
  while (key!='D'){
    if (key!=NO_KEY){
      newName = newName + key;
    }
    myGLCD.print(newName,LEFT, 140);
    if (key=='B'){
      editProcedure(newProcedure);
    }
    key = NO_KEY;
    while (key==NO_KEY){
      key = keypad.getKey();
    }
  } 
  if (newName.toInt() >32000){
    newName = 32000;
  }
  newProcedure->setName(newName.toInt());
  editProcedure(newProcedure);
}

void editNonStage(Procedure *newProcedure, Stage *currentStage){
  displayState = MENU_A;
  lastDisplayState = MENU_C;
  int menumax = MENU_C+1;

  vector<String> cetak;
  vector<String> cetakHasil;
  
  cetak.push_back("Time :");
  cetak.push_back("RPM :");
  
  cetakHasil.push_back((String)(currentStage->getTime()));
  cetakHasil.push_back((String)(currentStage->getRPM()));
  
  char key = keypad.getKey();
  
  while (key!='6'){
//    Serial.print(key);
    if(key=='8'){
      char currentState = static_cast<int>(displayState);
      displayState = static_cast<DisplayState>(++currentState % menumax);
    } else if (key == '2'){
      char currentState = static_cast<int>(displayState);
      displayState = static_cast<DisplayState>((currentState-1) < 0? static_cast<int>(menumax)-1 : --currentState);
    } 
    Serial.println(displayState);
    if (displayState != lastDisplayState){
      myGLCD.fillScr(40,85,154);
      myGLCD.setColor(255,255,255);
      myGLCD.setBackColor(40,85,154);
      myGLCD.print(stageName(currentStage->getName()), CENTER, 40);
      int piksel = 120;
      for (int i=0; i<cetak.size();i++){
        if (i == displayState){
          myGLCD.setBackColor(255,0,0);
          myGLCD.print(cetak[i], LEFT, piksel+(i*20));
          myGLCD.setBackColor(40,85,154);
        } else {
          myGLCD.setBackColor(40,85,154);
          myGLCD.print(cetak[i], LEFT, piksel+(i*20));
        }
        if (displayState == (menumax-1)){
          myGLCD.setBackColor(255,0,0);
          myGLCD.print("OK", LEFT, 300);
          myGLCD.setBackColor(40,85,154);
        } else {
          myGLCD.setBackColor(40,85,154);
          myGLCD.print("OK", LEFT, 300);
        }
      }
      piksel = 120;
      for (int i=0; i<cetakHasil.size();i++){
        myGLCD.print(cetakHasil[i], 200, piksel+(i*20), 0);
      }
      lastDisplayState = displayState;
    }
    if (key=='B'){
      break;
    } else if ((key=='D') && (displayState == menumax-1)){
      break;
    }
    key = NO_KEY;
    while (key==NO_KEY){
      key = keypad.getKey();
    }
  }

  if (key == '6'){
    switch(displayState){
      case MENU_A:
        myGLCD.setBackColor(40,85,154);
        myGLCD.setColor(255,255,255);
        myGLCD.print("Time :", LEFT, 120);
        editStageTime(currentStage);
        editNonStage(newProcedure, currentStage);
        break;
      case MENU_B:
        myGLCD.setBackColor(40,85,154);
        myGLCD.setColor(255,255,255);
        myGLCD.print("RPM :", LEFT, 140);
        editStageRPM(currentStage);
        editNonStage(newProcedure, currentStage);
        break;
      default:
        break;
    }
  }
}


void editStage(Procedure *newProcedure, Stage *currentStage){
  displayState = MENU_A;
  lastDisplayState = MENU_F;
  int menumax = lastDisplayState+1;

  vector<String> cetak;
  vector<String> cetakHasil;
  
  cetak.push_back("Fluid :");
  cetak.push_back("Time :");
  cetak.push_back("RPM :");
  cetak.push_back("Size :");
  cetak.push_back("Delete this stage");
  
  cetakHasil.push_back(fluidName(currentStage->getFluid()));
  cetakHasil.push_back((String)(currentStage->getTime()));
  cetakHasil.push_back((String)(currentStage->getRPM()));
  cetakHasil.push_back(sizeName(currentStage->getSize()));

  char key = keypad.getKey();
  
  while (key!='6'){
//    Serial.print(key);
    if(key=='8'){
      char currentState = static_cast<int>(displayState);
      displayState = static_cast<DisplayState>(++currentState % menumax);
    } else if (key == '2'){
      char currentState = static_cast<int>(displayState);
      displayState = static_cast<DisplayState>((currentState-1) < 0? static_cast<int>(menumax)-1 : --currentState);
    } 
    
    if (displayState != lastDisplayState){
      myGLCD.fillScr(40,85,154);
      myGLCD.setColor(255,255,255);
      myGLCD.setBackColor(40,85,154);
      myGLCD.print(stageName(currentStage->getName()), CENTER, 40);
      int piksel = 100;
      for (int i=0; i<cetak.size();i++){
        if (i == displayState){
          myGLCD.setBackColor(255,0,0);
          myGLCD.print(cetak[i], LEFT, piksel+(i*20));
          myGLCD.setBackColor(40,85,154);
        } else {
          myGLCD.setBackColor(40,85,154);
          myGLCD.print(cetak[i], LEFT, piksel+(i*20));
        }
        if (displayState == (menumax-1)){
          myGLCD.setBackColor(255,0,0);
          myGLCD.print("OK", LEFT, 300);
          myGLCD.setBackColor(40,85,154);
        } else {
          myGLCD.setBackColor(40,85,154);
          myGLCD.print("OK", LEFT, 300);
      }
      }
      piksel = 100;
      for (int i=0; i<cetakHasil.size();i++){
        myGLCD.print(cetakHasil[i], 200, piksel+(i*20), 0);
      }
      
      lastDisplayState = displayState;
//      Serial.println(displayState);
    }

    key = NO_KEY;
    while (key==NO_KEY){
      key = keypad.getKey();
    }
    if (key =='6'){
      if (displayState == MENU_F) {
        key = NO_KEY;
      } if (displayState == MENU_E){
        key = NO_KEY;
      }
    } else if (key == 'D'){
      if (displayState == 5){
        break;
      }
      else if (displayState == MENU_E){
        deleteStage(newProcedure, currentStage);
      }
    }
    else if (key =='B'){
      break;  
    }
  }
  
  if (key == '6'){
    switch(displayState){
      case MENU_A:
        myGLCD.setBackColor(40,85,154);
        myGLCD.setColor(255,255,255);
        myGLCD.print("Fluid :", LEFT, 100);
        editStageFluid(currentStage);
        editStage(newProcedure, currentStage);
        break;
      case MENU_B:
        myGLCD.setBackColor(40,85,154);
        myGLCD.setColor(255,255,255);
        myGLCD.print("Time :", LEFT, 120);
        editStageTime(currentStage);
        editStage(newProcedure, currentStage);
        break;
      case MENU_C:
        myGLCD.setBackColor(40,85,154);
        myGLCD.setColor(255,255,255);
        myGLCD.print("RPM :", LEFT, 140);
        editStageRPM(currentStage);
        editStage(newProcedure, currentStage);
        break;
      case MENU_D:
        myGLCD.setBackColor(40,85,154);
        myGLCD.setColor(255,255,255);
        myGLCD.print("Size :", LEFT, 160);
        editStageSize(currentStage);
        editStage(newProcedure, currentStage);
        break;
      default:
        break;
    }
  }
}

void deleteStage(Procedure *newProcedure, Stage *currentStage){
  allStage.clear();
  for (int i=0; i<newProcedure->getNumberStage()-1; i++){
    if (newProcedure->getStage()[i].getName() != currentStage->getName()){
      allStage.push_back(newProcedure->getStage()[i]);
    }
  }
  Stage tempStage[allStage.size()];
  for (int i=0; i<allStage.size(); i++){
    tempStage[i] = allStage[i];
  }
  Procedure changedProcedure = Procedure(newProcedure->getName(), allStage.size(), tempStage);
  *newProcedure = changedProcedure;
  editProcedure(newProcedure);
}

void editStageFluid(Stage *currentStage){
  displayState = MENU_A;
  lastDisplayState = MENU_E;
  myGLCD.setColor(40,85,154);
  myGLCD.fillRoundRect(200,100,480,119);
  
  myGLCD.setBackColor(40,85,154);
  myGLCD.setColor(255,255,255);
  
  myGLCD.setBackColor(255,0,0);
  myGLCD.print("SPM", 200, 100, 0);
  currentStage->setFluid(1);
  
  char key = keypad.getKey();
  while (key==NO_KEY){
    key = keypad.getKey();
  }
  int menumax = MENU_E+1;
  while (key!='4'){ 
    if(key=='8'){
      char currentState = static_cast<int>(displayState);
      displayState = static_cast<DisplayState>(++currentState % menumax);
    } else if (key == '2'){
      char currentState = static_cast<int>(displayState);
      displayState = static_cast<DisplayState>((currentState-1) < 0? static_cast<int>(menumax)-1 : --currentState);
    } 
    if(displayState != lastDisplayState)
    {
      switch(displayState){
        case MENU_A:
          myGLCD.setColor(40,85,154);
          myGLCD.fillRoundRect(200,100,480,119);
          myGLCD.setColor(255,255,255);
          myGLCD.setBackColor(255,0,0);
          myGLCD.print("SPM", 200, 100, 0);
          currentStage->setFluid(1);
          break;
        case MENU_B:
          myGLCD.setColor(40,85,154);
          myGLCD.fillRoundRect(200,100,480,119);
          myGLCD.setColor(255,255,255);
          myGLCD.setBackColor(255,0,0);
          myGLCD.print("Acetone", 200, 100, 0);
          currentStage->setFluid(2);
          break;
        case MENU_C:
          myGLCD.setColor(40,85,154);
          myGLCD.fillRoundRect(200,100,480,119);
          myGLCD.setColor(255,255,255);
          myGLCD.setBackColor(255,0,0);
          myGLCD.print("APM", 200, 100, 0);
          currentStage->setFluid(3);
          break;
        case MENU_D:
          myGLCD.setColor(40,85,154);
          myGLCD.fillRoundRect(200,100,480,119);
          myGLCD.setColor(255,255,255);
          myGLCD.setBackColor(255,0,0);
          myGLCD.print("DHF", 200, 100, 0);
          currentStage->setFluid(4);
          break;
        case MENU_E:
          myGLCD.setColor(40,85,154);
          myGLCD.fillRoundRect(200,100,480,119);
          myGLCD.setColor(255,255,255);
          myGLCD.setBackColor(255,0,0);
          myGLCD.print("HPM", 200, 100, 0);
          currentStage->setFluid(5);
          break;
    }
    lastDisplayState = displayState;
  }
    key = NO_KEY;
    while (key==NO_KEY){
      key = keypad.getKey();
    }
  }

  if (key == '4'){
    
  }
}

void editStageRPM(Stage *currentStage){
  myGLCD.setColor(40,85,154);
  myGLCD.fillRoundRect(200,140,480,159);
  myGLCD.setBackColor(255,0,0);
  int Number = 0;
  myGLCD.setColor(255,255,255);
  
  myGLCD.print((String)(Number),200,140,0);
  
  boolean result = false;
  char key = keypad.getKey();
  while (key==NO_KEY){
    key = keypad.getKey();
  }
  while (!result){
    if (key == '1'){ //If Button 1 is pressed
      if (Number==0)
        Number=1;
      else
        Number = (Number*10) + 1; //Pressed twice
      }
    
    if (key == '4'){ //If Button 4 is pressed
      if (Number==0)
        Number=4;
      else
        Number = (Number*10) + 4; //Pressed twice
    }
    
    if (key == '7'){ //If Button 7 is pressed
      if (Number==0)
        Number=7;
      else
        Number = (Number*10) + 7; //Pressed twice
    } 
    
    if (key == '0'){
      if (Number==0)
        Number=0;
      else
        Number = (Number*10) + 0; //Pressed twice
    }
    
    if (key == '2'){ //Button 2 is Pressed
      if (Number==0)
        Number=2;
      else
        Number = (Number*10) + 2; //Pressed twice
    }
    
    if (key == '5'){
      if (Number==0)
        Number=5;
      else
        Number = (Number*10) + 5; //Pressed twice
    }
    
    if (key == '8'){ 
      if (Number==0)
        Number=8;
      else
        Number = (Number*10) + 8; //Pressed twice
    }   

    if (key == '3'){
      if (Number==0)
        Number=3;
      else
        Number = (Number*10) + 3; //Pressed twice
    }
    
    if (key == '6'){
      if (Number==0)
        Number=6;
      else
        Number = (Number*10) + 6; //Pressed twice
    }
    
    if (key == '9'){
      if (Number==0)
        Number=9;
      else
        Number = (Number*10) + 9; //Pressed twice
    }  
    
    if (key == 'A'){
      Number = 0;
    }
    
    if (Number>32000){
      Number=32000;
    }
    
    if (key == 'D'){
      result = true;
    }
    myGLCD.setColor(40,85,154);
    myGLCD.fillRoundRect(200,140,480,159);
    myGLCD.setColor(255,255,255);
    myGLCD.print((String)(Number),200,140,0);    
    key = NO_KEY;
    if (!result){
      while (key==NO_KEY){
        key = keypad.getKey();
      }
    }
  }
  currentStage->setRPM(Number);
}

void editStageTime(Stage *currentStage){
  myGLCD.setColor(40,85,154);
  myGLCD.fillRoundRect(200,120,480,139);
  myGLCD.setBackColor(255,0,0);
  int Number = 0;
  myGLCD.setColor(255,255,255);
  
  
  myGLCD.print((String)(Number),200,120,0);
    
  boolean result = false;
  char key = keypad.getKey();
  while (key==NO_KEY){
    key = keypad.getKey();
  }
  while (!result){
    if (key == '1'){ //If Button 1 is pressed
      if (Number==0)
        Number=1;
      else
        Number = (Number*10) + 1; //Pressed twice
      }
    
    if (key == '4'){ //If Button 4 is pressed
      if (Number==0)
        Number=4;
      else
        Number = (Number*10) + 4; //Pressed twice
    }
    
    if (key == '7'){ //If Button 7 is pressed
      if (Number==0)
        Number=7;
      else
        Number = (Number*10) + 7; //Pressed twice
    } 
    
    if (key == '0'){
      if (Number==0)
        Number=0;
      else
        Number = (Number*10) + 0; //Pressed twice
    }
    
    if (key == '2'){ //Button 2 is Pressed
      if (Number==0)
        Number=2;
      else
        Number = (Number*10) + 2; //Pressed twice
    }
    
    if (key == '5'){
      if (Number==0)
        Number=5;
      else
        Number = (Number*10) + 5; //Pressed twice
    }
    
    if (key == '8'){ 
      if (Number==0)
        Number=8;
      else
        Number = (Number*10) + 8; //Pressed twice
    }   

    if (key == '3'){
      if (Number==0)
        Number=3;
      else
        Number = (Number*10) + 3; //Pressed twice
    }
    
    if (key == '6'){
      if (Number==0)
        Number=6;
      else
        Number = (Number*10) + 6; //Pressed twice
    }
    
    if (key == '9'){
      if (Number==0)
        Number=9;
      else
        Number = (Number*10) + 9; //Pressed twice
    }  

    if (key == 'A'){
      Number = 0;
    }
    
    if (Number>9000){
      Number=9000;
    }
    if (key == 'D'){
      result = true;
    }
    myGLCD.setColor(40,85,154);
    myGLCD.fillRoundRect(200,120,480,139);
    myGLCD.setColor(255,255,255);
    myGLCD.print((String)(Number),200,120,0);
    
    key = NO_KEY;
    if (!result){
      while (key==NO_KEY){
        key = keypad.getKey();
      }
    }
  }
  currentStage->setTime(Number);
}

void editStageSize(Stage *currentStage){
  displayState = MENU_A;
  lastDisplayState = MENU_B;
  myGLCD.setBackColor(255,0,0);
  myGLCD.setColor(255,255,255);
  myGLCD.print("3 x 3", 200, 160, 0);
  currentStage->setSize(1);
  
  char key = keypad.getKey();
  while (key==NO_KEY){
    key = keypad.getKey();
  }
  int menumax = MENU_B+1;
  while (key!='4'){ 
    if(key=='8'){
      char currentState = static_cast<int>(displayState);
      displayState = static_cast<DisplayState>(++currentState % menumax);
    } else if (key == '2'){
      char currentState = static_cast<int>(displayState);
      displayState = static_cast<DisplayState>((currentState-1) < 0? static_cast<int>(menumax)-1 : --currentState);
    } 
    if(displayState != lastDisplayState)
    {
      switch(displayState){
        case MENU_A:
          myGLCD.setColor(40,85,154);
          myGLCD.fillRoundRect(200,160,480,179);
          myGLCD.setColor(255,255,255);
          myGLCD.print("2 x 2", 200, 160, 0);
          currentStage->setSize(2);
          break;
        case MENU_B:
          myGLCD.setColor(40,85,154);
          myGLCD.fillRoundRect(200,160,480,179);
          myGLCD.setColor(255,255,255);
          myGLCD.print("3 x 3", 200, 160, 0);
          currentStage->setSize(1);
          break;
    }
    lastDisplayState = displayState;
  }
    key = NO_KEY;
    while (key==NO_KEY){
      key = keypad.getKey();
    }
  }

  if (key == '4'){
    
  }
}

void saveProcedure(){
  Serial.print("A");
  myGLCD.fillScr(40,85,154);
  myGLCD.setColor(255,255,255);
  myGLCD.print("Mohon menunggu", CENTER, 140);
  myGLCD.print("Sedang dilakukan", CENTER, 160);
  myGLCD.print("penyimpanan prosedur", CENTER, 180);
  for (int i=0; i<EEPROM.length(); i++){
    EEPROM.write(i,0);
  }
  savedProcedure.clear();
  savedProcedure.push_back(allProcedure.size());
  Serial.print("B");
  Serial.print("All Procedure Size :");
  Serial.println(allProcedure.size());
  for (int i=0; i<savedProcedure[0]; i++){
    savedProcedure.push_back(allProcedure[i].getName());
    Serial.print("Procedure Name :");
    Serial.println(allProcedure[i].getName());
    savedProcedure.push_back(allProcedure[i].getNumberStage());
    Serial.print("Get Number Stage :");
    Serial.println(allProcedure[i].getNumberStage());
    for (int j=0; j<allProcedure[i].getNumberStage(); j++){
      savedProcedure.push_back(allProcedure[i].getStage()[j].getName());
      Serial.print("Get Name :");
      Serial.println(allProcedure[i].getStage()[j].getName());
      savedProcedure.push_back(allProcedure[i].getStage()[j].getFluid());
      Serial.print("Get Fluid :");
      Serial.println(allProcedure[i].getStage()[j].getFluid());
      savedProcedure.push_back(allProcedure[i].getStage()[j].getRPM());
      Serial.print("Get RPM :");
      Serial.println(allProcedure[i].getStage()[j].getRPM());
      savedProcedure.push_back(allProcedure[i].getStage()[j].getTime());
      Serial.print("Get Time :");
      Serial.println(allProcedure[i].getStage()[j].getTime());
      savedProcedure.push_back(allProcedure[i].getStage()[j].getSize());
      Serial.print("Get Size :");
      Serial.println(allProcedure[i].getStage()[j].getSize());
    }
  }
  for (int i=0; i<savedProcedure.size(); i++){
    arraySaveProcedure[i] = savedProcedure[i];
  }
  arraySaveProcedure[savedProcedure.size()] = 255;
  EEPROM.put(0, arraySaveProcedure);
  for (int i=0; i<=savedProcedure.size(); i++){
    arraySaveProcedure[i] = 0;
  }
}


void loadProcedure(){
  allProcedure.clear();
  EEPROM.get(0, arraySaveProcedure);
  int i=0;
  int totalProcedure = arraySaveProcedure[i];
  Serial.print("Total Procedure :");
  Serial.println(totalProcedure);
  i++;
  int k=0;
  int nameProcedure;
  int totalStage;
  int nameStage;
  int fluidStage;
  int rpmStage;
  int timeStage;
  int sizeStage;
  while (arraySaveProcedure[i]!=255){
    if (arraySaveProcedure[i+1]==255){
      break;
    }
    nameProcedure = arraySaveProcedure[i];
    Serial.print("Procedure Name :");
    Serial.println(arraySaveProcedure[i]);
    i++;
    if (arraySaveProcedure[i]==255){
      break;
    }
    totalStage = arraySaveProcedure[i];
    Serial.print("Total Stage :");
    Serial.println(arraySaveProcedure[i]);
    i++;
    if (arraySaveProcedure[i]==255){
      break;
    }
    allStage.clear();
    for (int j=0; j<totalStage; j++){
      nameStage = arraySaveProcedure[i];
      Serial.print("Stage Name :");
      Serial.println(arraySaveProcedure[i]);
      i++;
      if (arraySaveProcedure[i]==255){
        break;
      }
      fluidStage = arraySaveProcedure[i];
      Serial.print("Stage Fluid :");
      Serial.println(arraySaveProcedure[i]);
      i++;
      if (arraySaveProcedure[i]==255){
        break;
      }
      rpmStage = arraySaveProcedure[i];
      Serial.print("Stage RPM :");
      Serial.println(arraySaveProcedure[i]);
      i++;
      if (arraySaveProcedure[i]==255){
        break;
      }
      timeStage = arraySaveProcedure[i];
      Serial.print("Stage Time :");
      Serial.println(arraySaveProcedure[i]);
      i++;
      if (arraySaveProcedure[i]==255){
        break;
      }
      sizeStage = arraySaveProcedure[i];
      Serial.print("Stage Size :");
      Serial.println(arraySaveProcedure[i]);
      i++;
      if (arraySaveProcedure[i]==255){
        break;
      }
      Stage newStage = Stage(nameStage, fluidStage, rpmStage, timeStage, sizeStage);
//      Serial.println("Actual Stage");
//      Serial.print("Stage Name : ");
//      Serial.println(nameStage);
//      Serial.print("Stage Fluid :");
//      Serial.println(fluidStage);
//      Serial.print("Stage RPM : ");
//      Serial.println(rpmStage);
//      Serial.print("Stage Time :");
//      Serial.println(timeStage);
//      Serial.print("Stage Size : ");
//      Serial.println(sizeStage);
      allStage.push_back(newStage);
    }
    Stage tempStage[allStage.size()];
    for (int j=0; j<allStage.size(); j++){
      tempStage[j] = allStage[j];
      Serial.println("Actual Stage");
      Serial.print("Stage Name : ");
      Serial.println(tempStage[j].getName());
      Serial.print("Stage Fluid :");
      Serial.println(tempStage[j].getFluid());
      Serial.print("Stage RPM : ");
      Serial.println(tempStage[j].getRPM());
      Serial.print("Stage Time :");
      Serial.println(tempStage[j].getTime());
      Serial.print("Stage Size : ");
      Serial.println(tempStage[j].getSize());
    }
    Procedure new_procedure = Procedure(nameProcedure, totalStage, tempStage);
    allProcedure.push_back(new_procedure);
  }
}

void AloadProcedure(){
  allProcedure.clear();
  EEPROM.get(0, arraySaveProcedure);
  int i=0;
  Serial.print("Total Procedure :");
  Serial.println(arraySaveProcedure[i]);
  i++;
  while (arraySaveProcedure[i]!=255){
    if (arraySaveProcedure[i+1]==255){
      break;
    }
    Serial.print("Procedure Name :");
    Serial.println(arraySaveProcedure[i]);
    i++;
    if (arraySaveProcedure[i]==255){
      break;
    }
    Serial.print("Total Stage :");
    int totalStage = arraySaveProcedure[i];
    Serial.println(arraySaveProcedure[i]);
    i++;
    for (int j=0; j<totalStage; j++){
      Serial.print("Stage Name :");
      Serial.println(arraySaveProcedure[i]);
      i++;
      if (arraySaveProcedure[i]==255){
        break;
      }
      Serial.print("Stage Fluid :");
      Serial.println(arraySaveProcedure[i]);
      i++;
      if (arraySaveProcedure[i]==255){
        break;
      }
      Serial.print("Stage RPM :");
      Serial.println(arraySaveProcedure[i]);
      i++;
      if (arraySaveProcedure[i]==255){
        break;
      }
      Serial.print("Stage Time :");
      Serial.println(arraySaveProcedure[i]);
      i++;
      if (arraySaveProcedure[i]==255){
        break;
      }
      Serial.print("Stage Size :");
      Serial.println(arraySaveProcedure[i]);
      i++;
      
    }
  }
}
ISR(TIMER1_COMPA_vect){
    //interrupt commands here
    White_count1();
    pwm_now = PID_Motor (desired_speed, actual_speed, pwm_now);
    //Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    
    // Calculating the distance
    distanceStepper = duration*34/2;
//    Serial.println(distanceStepper);
    // Prints the distance on the Serial Monitor
//    Serial.print("Distance: ");
//    Serial.println(distanceStepper);
}

int White_count1 () {
    actual_speed = count1 * 60;
    count1=0;
    //Serial.println(pwm_now);
    //Serial.println(motor_speed);
}

int Rpm_Sensor () { //TCRCT5000 as RPM Sensor Arduino Code
    count1 += 1;
}

int PID_Motor (int desired_speed, int actual_speed, int pwm_now) {
  int error0;
  float result;
  float kp;
  int result_pwm;
//
// kp = 0.007;
//  ki = 0.002;
//  kd = 0.003;
//
//  error0 = desired_speed - actual_speed;
//  error1 = (error0 - lasterror)/1;
//  error2 = error2 + error0*1;
//
////  Serial.print("error0 = "); Serial.print(error0); Serial.print(" ");
//  if ((error0 <= 100) && (error0 > -100)) {
//  result_pwm = pwm_now;
//  } else {
//  result = kp*error0 + kd*error1 + ki*error2;
//  result_pwm = int(result);
//  }
//
//  lasterror = error0;
//  
  kp = 0.001;

  error0 = desired_speed - actual_speed;
  result = kp*error0;

  if (result > 0.3) {
    result_pwm = pwm_now + 3;
  } else if ((result > 0.2) && (result <= 0.3)) {
    result_pwm = pwm_now + 2;
  } else if ((result > 0.05) && (result <= 0.2)) {
    result_pwm = pwm_now + 1;
  } else if ((result > -0.05 ) && (result <= 0.05)) {
    result_pwm = pwm_now;
  } else if ((result > -0.2 ) && (result <= -0.05)) {
    result_pwm = pwm_now - 1;
  } else if ((result > -0.3 ) && (result <= -0.2)) {
    result_pwm = pwm_now - 2;
  } else {
    result_pwm = pwm_now - 3;
  }
  
  return result_pwm;
}

void set_timer1 () {
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 3900; //31284;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}
