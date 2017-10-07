#include <Wire.h>
#include "Adafruit_MCP9808.h"
#include "PID_v1.h"
#include "PID_AutoTune_v0.h"

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

byte ATuneModeRemember=2;
double input=25, output=50, setpoint=5;
// Se obtuvo del autotuneo
double kp=328.09,ki=2.35,kd=11433.87;

double kpmodel=1.5, taup=100, theta[50];
double outputStart=-127;
double aTuneStep=127, aTuneNoise=0.2, aTuneStartValue=-127;
unsigned int aTuneLookBack=20;

boolean tuning = true;
unsigned long  modelTime, serialTime;

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&input, &output);

//set to false to connect to the real world
boolean useSimulation = false;

void setup()
{

  int pincalentado = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
  pinMode(pincalentado, OUTPUT);
  if(useSimulation)
  {
    for(byte i=0;i<50;i++)
    {
      theta[i]=outputStart;
    }
    modelTime = 0;
  }
  //Setup the pid
  myPID.SetMode(AUTOMATIC);

  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }

  serialTime = 0;
  Serial.begin(9600);

}

void loop()
{
  unsigned long now = millis();

  if(!useSimulation)
  { //pull the input in from the real world
    if (!tempsensor.begin(0x1B)) {
      Serial.println("Sensor 0x1B no encontrado");
      while (1);
    }
    else {
      tempsensor.wake();
      float c = tempsensor.readTempC();
      tempsensor.shutdown();
      input = c;
    }
  }

  if(tuning)
  {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      Serial.println("Ahi van las salidas del autotune");
      Serial.println(kp);
      Serial.println(ki);
      Serial.println(kd);
      myPID.SetTunings(kp,ki,kd);
      //Agrego un tiempo de control mayor a 0.1s
      myPID.SetSampleTime(1000);
      AutoTuneHelper(false);
    }
  }
  else myPID.Compute();

  if(useSimulation)
  {
    theta[30]=output;
    if(now>=modelTime)
    {
      modelTime +=100;
      DoModel();
    }
  }
  else
  {
    int pincalentado = 6;
    int pinenfriado = 5;
    if (output >= 0) {
    analogWrite(pinenfriado,0);
    analogWrite(pincalentado,output);
    }
    if (output < 0) {
    analogWrite(pincalentado,0);
    analogWrite(pinenfriado,-1*output);
    }
    //Para almacenar el valor de potencia
    Serial.print("[5, ");
    Serial.print(output);
    Serial.print("]\n");
  }

  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }

  int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
  int LPWM_Output = 6;
  int intervalo = 10;
  // Lo siguiente deberia ser reemplazado por un loop

  if (!tempsensor.begin(0x18)) {
    Serial.println("Sensor 0x18 no encontrado");
    while (1);
  }
  else {
    tempsensor.wake();
    float c = tempsensor.readTempC();
    Serial.print("[0, ");
    Serial.print(c);
    Serial.print("]\n");
    if (c > 70)
    {
      analogWrite(LPWM_Output, 0);
      analogWrite(RPWM_Output, 0);
    }
    tempsensor.shutdown();
  }
  delay(10);

  if (!tempsensor.begin(0x19)) {
    Serial.println("Sensor 0x19 no encontrado");
    while (1);
  }
  else {
    tempsensor.wake();
    float c = tempsensor.readTempC();
    Serial.print("[1, ");
    Serial.print(c);
    Serial.print("]\n");
    if (c > 70)
    {
      analogWrite(LPWM_Output, 0);
      analogWrite(RPWM_Output, 0);
    }
    tempsensor.shutdown();
  }
  delay(10);

  if (!tempsensor.begin(0x1A)) {
    Serial.println("Sensor 0x1A no encontrado");
    while (1);
  }
  else {
    tempsensor.wake();
    float c = tempsensor.readTempC();
    Serial.print("[2, ");
    Serial.print(c);
    Serial.print("]\n");
    if (c > 70)
    {
      analogWrite(LPWM_Output, 0);
      analogWrite(RPWM_Output, 0);
    }
    tempsensor.shutdown();
  }
  delay(10);

  if (!tempsensor.begin(0x1B)) {
    Serial.println("Sensor 0x1B no encontrado");
    while (1);
  }
  else {
    tempsensor.wake();
    float c = tempsensor.readTempC();
    Serial.print("[3, ");
    Serial.print(c);
    Serial.print("]\n");
    if (c > 70)
    {
      analogWrite(LPWM_Output, 0);
      analogWrite(RPWM_Output, 0);
    }
    tempsensor.shutdown();
  }
  delay(10);

  if (!tempsensor.begin(0x1C)) {
    Serial.println("Sensor 0x1C no encontrado");
    while (1);
  }
  else {
    tempsensor.wake();
    float c = tempsensor.readTempC();
    Serial.print("[4, ");
    Serial.print(c);
    Serial.print("]\n");
    tempsensor.shutdown();
  }
  delay(10);
}

void changeAutoTune()
{
  if(!tuning)
  {
    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}


void SerialSend()
{
  Serial.print("setpoint: ");
  Serial.print(setpoint);
  Serial.print(" ");
  Serial.print("input: ");
  Serial.print(input);
  Serial.print(" ");
  Serial.print("output: ");
  Serial.print(output);
  Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  }
  else {
    Serial.print("kp: ");
    Serial.print(myPID.GetKp());
    Serial.print(" ");
    Serial.print("ki: ");
    Serial.print(myPID.GetKi());
    Serial.print(" ");
    Serial.print("kd: ");
    Serial.print(myPID.GetKd());
    Serial.println();
  }
}

void SerialReceive()
{
  if(Serial.available())
  {
    char b = Serial.read();
    Serial.flush();
    if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}

void DoModel()
{
  //cycle the dead time
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  //compute the input
  input = (kpmodel / taup) *(theta[0]-outputStart) + input*(1-1/taup) + ((float)random(-10,10))/100;

}
