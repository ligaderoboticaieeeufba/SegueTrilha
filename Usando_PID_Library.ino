#include <QTRSensors.h>
#include <PID_v1.h>

#define NUM_SENSORS   6     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define M1F 10 //pra frente com roda 1
#define M1T 9 // pra trás com roda 1
#define M2F 11  //pra frente com roda 2
#define M2T 3   // pra trás com roda 2
#define velocidade 230
#define vel_calibracao 135
#define Calibracao 2
#define Rodar 4

unsigned long interval=400; // the time we need to wait
unsigned long previousMillis=0; // millis() returns an unsigned long.
unsigned long previousMillis2=0; // millis() returns an unsigned long.
int Motor1[2] = {M1F,M1T};
int Motor2[2] = {M2F,M2T};
  int m1=1;
  int m2=2;
  int buff;
int calibrador=0;
int fator=20;


//Configurando sensor
QTRSensorsRC qtrrc((unsigned char[]) {A0,A1,A2,A3,A4,A5},
  NUM_SENSORS, TIMEOUT); 
//======================================
unsigned int sensorValues[NUM_SENSORS];
//======================================
//Variaveis que vamos usar.
double Setpoint, Input, Output;
//======================================
//Ganhos e configuração do PID

double Kp = 0.16;//0,075  0,09  //0,1
double Ki=0.0000;//0
double Kd=0.25;//0,0024  0,002 0,0036//0,0036

//kp = 0.1 ki =0.001 kd = 0.002

//velocidade 90 => kp =0.075 ki =0 kd=0.002
//adaptativo => velocidade 200, kpaggr=0.17,kiaggr=0,kdaggr=0.0055 kpsuave =0.1 kisuave = 0, kdsuave=0.004
PID superPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//======================================

void setup() {
  pinMode(2,INPUT); // Botao Calibração
  pinMode(4,INPUT); //Botão Rodar o Carro
 // Serial.begin(9600);
   /*if(digitalRead(Calibracao)==HIGH)
 {*/
  int x=0;
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 200; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call
 //   calibracao(i,calibrador,fator);
 
      
  } 
    digitalWrite(13, LOW);    // turn off Arduino's LED to indicate we are in calibration mode
 //}
  unsigned int position = qtrrc.readLine(sensorValues);
  //Pega posição como Double pro PID
  Input = position;
  //Determina SetPoint
  Setpoint = 2500;
  //Define range na saída do controlador
  superPID.SetOutputLimits(-velocidade,velocidade);
  //Determina tempo de execução do PID - 10ms ou 100 execuções por segundo
  superPID.SetSampleTime(1); //em ms
  //Liga o PID
  superPID.SetMode(AUTOMATIC);
  superPID.SetTunings(Kp,Ki,Kd);
}

void loop() {
    unsigned int position = qtrrc.readLine(sensorValues);
    Input = position;

    //Serial.println(Output);
    //delay(100);
    
    
    superPID.Compute();
    
    
    if(Output>2500)
    {
      
      analogWrite(M1F,velocidade-Output);
      analogWrite(M2F,velocidade+Output);
    
    }
    else if(Output<2500)
    {
      analogWrite(M1F,velocidade+Output);
      analogWrite(M2F,velocidade-Output);
      
    }
    else
    {
      analogWrite(M1F,velocidade);
      analogWrite(M2F,velocidade);
    }

    
  

    

 //  unsigned long currentMillis = millis(); // grab current time
 
 // check if "interval" time has passed (1000 milliseconds)
 /*
 if ((unsigned long)(currentMillis - previousMillis) >= interval) {
  analogWrite(Motor1[m1],vel_calibracao);
       analogWrite(Motor2[m2],vel_calibracao);
   Serial.print(Output);
   Serial.print("  ");
   Serial.println(Input);
  
   // save the "current" time
   previousMillis = millis();
 }
    */
}

void calibracao(int i,int fator,int calibrador){
  
   Serial.println(m1);
  if(i<20){
       analogWrite(M1F,vel_calibracao);
       analogWrite(M2T,vel_calibracao);
       analogWrite(M1T,0);
       analogWrite(M2F,0);
}
else if(i>=20&& i<40){
       analogWrite(M1T,0);
       analogWrite(M2F,0);
       analogWrite(M1F,vel_calibracao);
       analogWrite(M2T,vel_calibracao);
       

}
else if(i>=40 && i<60){
         analogWrite(M1T,0);
       analogWrite(M2F,0);
       analogWrite(M1F,vel_calibracao);
       analogWrite(M2T,vel_calibracao);

}
else if(i>=60&& i<80){
       analogWrite(M1T,0);
       analogWrite(M2F,0);
       analogWrite(M1F,vel_calibracao);
       analogWrite(M2T,vel_calibracao);
       
}
else if(i>=80 && i<100){
         analogWrite(M1T,0);
       analogWrite(M2F,0);
       analogWrite(M1F,vel_calibracao);
       analogWrite(M2T,vel_calibracao);

}
else if(i>=100&& i<120){
       analogWrite(M1T,0);
       analogWrite(M2F,0);
       analogWrite(M1F,vel_calibracao);
       analogWrite(M2T,vel_calibracao);
 
}
else if(i>=120 && i<140){
         analogWrite(M1T,0);
       analogWrite(M2F,0);
       analogWrite(M1F,vel_calibracao);
       analogWrite(M2T,vel_calibracao);

}
else if(i>=140&& i<160){
       analogWrite(M1T,0);
       analogWrite(M2F,0);
       analogWrite(M1F,vel_calibracao);
       analogWrite(M2T,vel_calibracao);
}
else if(i>=160 && i<180){
         analogWrite(M1T,0);
       analogWrite(M2F,0);
       analogWrite(M1F,vel_calibracao);
       analogWrite(M2T,vel_calibracao);

}
else if(i>=180 && i<199){
       analogWrite(M1T,0);
       analogWrite(M2F,0);
       analogWrite(M1F,vel_calibracao);
       analogWrite(M2T,vel_calibracao);
}
else{
        analogWrite(M1T,0);
       analogWrite(M2F,0);
       analogWrite(M1F,0);
       analogWrite(M2T,0);
}
}





