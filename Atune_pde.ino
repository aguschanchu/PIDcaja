#include <Wire.h>
#include "Adafruit_MCP9808.h"
#include "PID_v1.h"
#include "PID_AutoTune_v0.h"

////////Ajustes
double kp=302.61,ki=1.08,kd=21121.99;
double setpoint=40;
int temperaturaMaxima=71;
boolean tuning = true;
// Utiliza el resistor como perturbacion
boolean perturbador = true;
//Settings del perturbador
double dutyCyclePerturbacion = 0.4;
int periodoPerturbacion = 15; //en minutos
int esperaInicialPerturbado = 60; //Cuanto espera antes de iniciar el perturbador en minutos
//Settings de los pins del puente H
int pincalentado = 5;
int pinenfriado = 6;

////////Fin de ajustes
// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
byte ATuneModeRemember=2;
double input=25, output=0;
double outputStart=0;
double aTuneStep=255, aTuneNoise=0.2, aTuneStartValue=0;
unsigned int aTuneLookBack=20;
unsigned long ultimoPeriodo = -periodoPerturbacion*60*100;
unsigned long contadorEsperaInicial = 0;
bool dutyModificacion = false;
unsigned long  modelTime, serialTime;
PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&input, &output, &setpoint);


void setup()
{
  int R_en = 2;
  int L_en = 1;
  pinMode(pincalentado, OUTPUT);
  pinMode(pinenfriado, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(R_en, OUTPUT);
  pinMode(L_en, OUTPUT);
  digitalWrite(R_en, 1);
  digitalWrite(L_en, 1);
  digitalWrite(7, 1);

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
  // Reloj en segundos
  unsigned long tiempo = now/1000;

  //pull the input in from the real world
  if (!tempsensor.begin(0x18)) {
    Serial.println("Sensor 0x18 no encontrado");
    while (1);
  }
  else {
    tempsensor.wake();
    double c;
     if (tuning) {
      // Hay que tomar menos puntos si promediamos, por el NLoookBack    if (tunning) {
        for (int j = 0; j < 10; j++) {
        c = c + tempsensor.readTempC();
        delay(100);
      }
      tempsensor.shutdown();
      input = c/10;
      }
    else {
      for (int j = 0; j < 500; j++) {
        c = c + tempsensor.readTempC();
        delay(20);
      }
      tempsensor.shutdown();
      input = c/500;
    }
    Serial.print("[7,");
    Serial.print(input);
    Serial.print(",");
    Serial.print(tiempo)
    Serial.print("]\n");
  }


  if(tuning)
  {
    byte val = (aTune.Runtime());
    //Si esta tuneado, no queremos perturbarlo, de modo que almacenamos
    //la intencion de hacerlo, en una variable, para despues reactivarlo
    if (perturbador){
      bool retenerPerturbador = true;
      perturbador = false;
    }
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      Serial.println("Ahi van las salidas del autotune, como estado");

      Serial.print("['e',");
      Serial.print(tiempo)
      Serial.print(",");
      Serial.print(kp);
      Serial.print(",");
      Serial.print(ki);
      Serial.print(",");
      Serial.print(kd);
      Serial.print(",");
      Serial.print(setpoint);
      Serial.print(",");
      Serial.print("'autotune finalizado, PIDando'");
      Serial.print("]\n");

      myPID.SetTunings(kp,ki,kd);
      //Agrego un tiempo de control mayor a 0.1s
      myPID.SetSampleTime(1000);
      AutoTuneHelper(false);
      //Reactivamos el perturbador en caso de que haya que hacerlo
      //y reiniciamos el contadorEsperaInicial
      contadorEsperaInicial=now;
      if (retenerPerturbador){
        perturbador=true;
      }


    }
  }
  else myPID.Compute();


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
  Serial.print("[6,");
  Serial.print(output);
  Serial.print(",");
  Serial.print(tiempo)
  Serial.print("]\n");

  // Perturbamos a la caja con el resistor
  if (perturbador && (now-contadorEsperaInicial)>esperaInicial*60*1000){
    if ((now-ultimoPeriodo)>=periodoPerturbacion*60*1000){
      CambiarVoltaje(6);
      dutyModificacion=false;
      ultimoPeriodo=now;
    }
    elseif ((now-ultimoPeriodo)>=periodoPerturbacion*60*1000*dutyCyclePerturbacion && dutyModificacion == false){
      CambiarVoltaje(3);
      dutyModificacion=true;
    }

  }
  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }

  // Enviamos la temperatura del resto de los sensores
  int numeroDeSensores = 6;
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
      Serial.print(",");
      Serial.print(tiempo)
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

void CambiarVoltaje(double voltaje)
{
  //Envia el voltaje en voltios al arduino, para que lo cambie en la fuente de DC
  Serial.print("['v',");
  Serial.print(voltaje);
  Serial.print(",");
  Serial.print(millis()/1000)
  Serial.print("]\n");

}
