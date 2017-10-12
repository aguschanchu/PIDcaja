#include <Wire.h>
#include "Adafruit_MCP9808.h"
#include "PID_v1.h"
#include "PID_AutoTune_v0.h"

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

byte ATuneModeRemember=2;
double input=25, output=50, setpoint=40;
// Se obtuvo del autotuneo
double kp=25.34,ki=0.19,kd=850.99;

double kpmodel=1.5, taup=100, theta[50];
double outputStart=0;
double aTuneStep=255, aTuneNoise=0.2, aTuneStartValue=0;
unsigned int aTuneLookBack=20;
int temperaturaMaxima=71;
boolean tuning = true;
unsigned long  modelTime, serialTime;

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&input, &output, &setpoint);


void setup()
{

  int pincalentado = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
  pinMode(pincalentado, OUTPUT);

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

  //pull the input in from the real world
  if (!tempsensor.begin(0x1B)) {
    Serial.println("Sensor 0x1B no encontrado");
    while (1);
  }
  else {
    tempsensor.wake();
    float c;
    for (int j = 0; j < 10; j++) {
      c = c + tempsensor.readTempC();
      delay(100);
    }
    tempsensor.shutdown();
    input = c/10;
    Serial.print("[6,");
    Serial.print(input);
    Serial.print("]\n");
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


  int pincalentado = 6;
  int pinenfriado = 5;
  //Ejecutamos una funcion que verifique que no se este friendo la caja
  if (input > temperaturaMaxima) {
    output=0;
  }

  //Aplicamos los cambios del PID
  if (output >= 0) {
  analogWrite(pinenfriado,0);
  analogWrite(pincalentado,output);
  }
  if (output < 0) {
  analogWrite(pincalentado,0);
  analogWrite(pinenfriado,-1*output);
  }

  //Para almacenar el valor de potencia
  Serial.print("[5,");
  Serial.print(output);
  Serial.print("]\n");


  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }

  // Enviamos la temperatura del resto de los sensores
  int numeroDeSensores = 5;
  for (int sensor = 24; sensor < 24 + numeroDeSensores; sensor++) {
    if (!tempsensor.begin(sensor)) {
      Serial.print("Sensor ");
      Serial.print(sensor-24);
      Serial.print("no encontrado\n");
      while (1);
    }
    else {
      tempsensor.wake();
      float c = tempsensor.readTempC();
      Serial.print("[");
      Serial.print(sensor-24);
      Serial.print(",");
      Serial.print(c);
      Serial.print("]\n");
      tempsensor.shutdown();
    }
    delay(10);
  }

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
