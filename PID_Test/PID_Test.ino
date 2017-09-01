#include <SparkFun_Si7021_Breakout_Library.h>
#include <Wire.h>
#include <PID_v1.h>


//PID Controller Variables
float Kp = 1;
float Ki = 10;
float Kd = 1;

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
const int sampleRate = 10;
const long serialPing =500;
unsigned long now = 0;
unsigned long lastMessage = 0;


float humidity = 0;
float tempC = 0;
int speedVal1 = 20;   //Speed value for exhaust fan
int speedVal2 = 0;    //Speed value for stimulation fan
int speedVal3 = 0;    //Speed value for input fan

long entry = 0;

//Change time of polling system in ms intervals.
unsigned long timeVal = 5000;  

char fanSpeed[13];

int fan1 = 3;   //Exhaust Fan
int fan2 = 5;   //Stimulation Fan
int fan3 = 6;   //Input Fan

Weather sensor;

//Function prototypes
void tempVal();
void printText();
void fanControlLoop();
void PID_init();
void PID_Control();


//Custom PID Code Variables
float current_position = 0;
float error = 0;
float integral = 0;
float derivative = 0;
float pwm_out = 0;
float target_position = 23.2;
float last_error = 0;
int x = 0;

float kp = 3;
float ki = 1;
float kd = 1;
void PID_Loop();


void setup() {

  Serial.begin(9600); //Serial port begin at 9600 baud

  //Pin mode setup//

  pinMode(fan1, OUTPUT);
  pinMode(fan2, OUTPUT);
  pinMode(fan3, OUTPUT);

  //Begin Silabs sensor
  sensor.begin();
  //PID_init();
}

void loop() {
  
  //tempVal();      //Get current temp value

  //fanControlLoop();
  
  //entry++;
  //printText();

  //PID_Control();
  PID_Loop();

  delay(timeVal); //Sampling delay
}

void tempVal(){
  humidity = sensor.getRH();  //Triggers reading temperature and humidity
  tempC = sensor.getTemp();   //Reading temperature value from last humidity reading
}

int fanSpeedCon(int a){

  if(a == 0){
     speedVal1 = a; //As the exhaust fan can't be off.
     speedVal2 = a; //Keeps stimulation fan running a speed because the input fan is off.
     speedVal3 = a;   //Input fan is off.
     
     analogWrite(fan1, speedVal1);
     analogWrite(fan2, speedVal2);
     analogWrite(fan3, speedVal3);    
  }
  else if(a == 70){
     speedVal1 = a; //As the exhaust fan can't be off.
     speedVal2 = a; //Keeps stimulation fan running a speed because the input fan is off.
     speedVal3 = a;   //Input fan is off.
         
     analogWrite(fan1, speedVal1);
     analogWrite(fan2, speedVal2);
     analogWrite(fan3, speedVal3);    
  }
  else if(a == 100){
     speedVal1 = a; //As the exhaust fan can't be off.
     speedVal2 = a; //Keeps stimulation fan running a speed because the input fan is off.
     speedVal3 = a;   //Input fan is off.
         
     analogWrite(fan1, speedVal1);
     analogWrite(fan2, speedVal2);
     analogWrite(fan3, speedVal3);    
  }
  else if(a == 130){
     speedVal1 = a; //As the exhaust fan can't be off.
     speedVal2 = 0; //Keeps stimulation fan running a speed because the input fan is off.
     speedVal3 = a;   //Input fan is off.
         
     analogWrite(fan1, speedVal1);
     analogWrite(fan2, speedVal2);
     analogWrite(fan3, speedVal3);    
  }      
  else if(a == 170){
     speedVal1 = a; //As the exhaust fan can't be off.
     speedVal2 = 0; //Keeps stimulation fan running a speed because the input fan is off.
     speedVal3 = a;   //Input fan is off.
         
     analogWrite(fan1, speedVal1);
     analogWrite(fan2, speedVal2);
     analogWrite(fan3, speedVal3);    
  }    
  else if(a == 255){
     speedVal1 = a; //As the exhaust fan can't be off.
     speedVal2 = 0; //Keeps stimulation fan running a speed because the input fan is off.
     speedVal3 = a;   //Input fan is off.
         
     analogWrite(fan1, speedVal1);
     analogWrite(fan2, speedVal2);
     analogWrite(fan3, speedVal3);    
  }  
}

/*int fanSpeedCon(int a){

  if(a == 0){
     speedVal1 = a+23; //As the exhaust fan can't be off.
     speedVal2 = a+73; //Keeps stimulation fan running a speed because the input fan is off.
     speedVal3 = a;   //Input fan is off.
     
     analogWrite(fan1, speedVal1);
     analogWrite(fan2, speedVal2);
     analogWrite(fan3, speedVal3);    
  }
  else if(a == 64){
     speedVal1 = a+35; //As the exhaust fan can't be off.
     speedVal2 = a-12; //Keeps stimulation fan running a speed because the input fan is off.
     speedVal3 = a;   //Input fan is off.
         
     analogWrite(fan1, speedVal1);
     analogWrite(fan2, speedVal2);
     analogWrite(fan3, speedVal3);    
  }
  else if(a == 192){
     speedVal1 = a+40; //As the exhaust fan can't be off.
     speedVal2 = a-152; //Keeps stimulation fan running a speed because the input fan is off.
     speedVal3 = a;   //Input fan is off.
         
     analogWrite(fan1, speedVal1);
     analogWrite(fan2, speedVal2);
     analogWrite(fan3, speedVal3);    
  }
  else if(a == 255){
     speedVal1 = a; //As the exhaust fan can't be off.
     speedVal2 = 0; //Keeps stimulation fan running a speed because the input fan is off.
     speedVal3 = a-25;   //Input fan is off.
         
     analogWrite(fan1, speedVal1);
     analogWrite(fan2, speedVal2);
     analogWrite(fan3, speedVal3);    
  }  
}*/

void printText(){
  Serial.print("Entry:");
  Serial.println(entry);
  Serial.print("Temp:");
  Serial.print(tempC);
  Serial.println("C");
  Serial.print("Humidity:");
  Serial.print(humidity);
  Serial.println("%");
  Serial.print("Current Fan Speed:");
  Serial.println(fanSpeed); 
  Serial.print("Fan 1 speed:");
  Serial.println(speedVal1); 
  Serial.print("Fan 2 speed:");
  Serial.println(speedVal2);
  Serial.print("Fan 3 speed:");
  Serial.print(speedVal3);        
  Serial.print("\n\n");
}

void fanControlLoop(){

    memset(fanSpeed, 0, sizeof(fanSpeed));
    
    /*if(tempC>=18 && tempC<22){
      fanSpeedCon(64); //25% duty cycle
      strcpy(fanSpeed, "Low");
    }
    else if(tempC>=22 && tempC<30){
       fanSpeedCon(192); //75% duty cycle
       strcpy(fanSpeed, "Medium");
    }
    else if(tempC<18){
       fanSpeedCon(0); //0% duty cycle
       strcpy(fanSpeed, "Off");
    }
    else if(tempC>=30){
       fanSpeedCon(255);  //100% duty cycle
       strcpy(fanSpeed, "High");
    } */

    if(tempC>=21 && tempC<22){
      fanSpeedCon(70); //25% duty cycle
      strcpy(fanSpeed, "Low");
    }
    else if(tempC>=22 && tempC<23){
       fanSpeedCon(100); //75% duty cycle
       strcpy(fanSpeed, "Low-Medium");
    }  
    else if(tempC>=23 && tempC<24){
       fanSpeedCon(130); //75% duty cycle
       strcpy(fanSpeed, "Medium");
    }
    else if(tempC>=24 && tempC<25){
       fanSpeedCon(170); //75% duty cycle
       strcpy(fanSpeed, "Medium-High");
    }
    else if(tempC<21){
       fanSpeedCon(0); //0% duty cycle
       strcpy(fanSpeed, "Off");
    }
    else if(tempC>=25){
       fanSpeedCon(255);  //100% duty cycle
       strcpy(fanSpeed, "Max!");
    }     
}

void PID_init()
{
  Input = sensor.getTemp();
  Setpoint = 17.5;

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(sampleRate);

    Serial.println("Begin");

    lastMessage = millis();
}

void PID_Control()
{
  Input = sensor.getTemp();
  myPID.Compute();

  analogWrite(fan1, Output);

  now = millis();

  if(now - lastMessage > serialPing){
    Serial.print("Setpoint = ");
    Serial.print(Setpoint);
    Serial.print(" Input = ");
    Serial.print(Input);
    Serial.print(" Output = ");
    Serial.print(Output);
    Serial.print("\n");

    if(Serial.available() > 0){
      for(int x = 0; x < 4; x++){
        switch(x){
          case 0:
            Kp = Serial.parseFloat();
            break;
          case 1:
            Ki = Serial.parseFloat();
            break;
          case 2:
            Kd = Serial.parseFloat();
            break;
          case 3:
            for (int y = Serial.available(); y == 0; y--){
                Serial.read();  
            }
            break;
        }
      }
      Serial.print(" Kp,Ki,Kd =");
      Serial.print(Kp);
      Serial.print(",");
      Serial.print(Ki);
      Serial.print(",");
      Serial.println(Kd);
      myPID.SetTunings(Kp, Ki, Kd);
    }
    lastMessage = now;
  }
}

void PID_Loop()
{
  float pwm_pos = 0;
  int Mot_PWM = 0;
  current_position = sensor.getTemp();

  error = target_position - current_position;

  integral = integral + error;

  derivative = error - last_error;

  pwm_out = (kp * error) + (ki * integral) + (kd * derivative);

  if(pwm_out>255)
  {
    pwm_out = 255;
  }
  else if(pwm_out < -255)
  {
    pwm_out = -255;
  }

  if(integral > 0)
  {
    integral = 0;
  }
  x++;
  Serial.print("Test Value: ");
  Serial.println(x);
  Serial.print("Value of current temperature is: ");
  Serial.println(current_position);
  Serial.print("Value of error is: ");
  Serial.println(error);
  Serial.print("Value of integral is: ");
  Serial.println(integral);
  Serial.print("Value of derivative is: ");
  Serial.println(derivative);
  Serial.print("Value of output is: ");
  Serial.println(pwm_out);
  Serial.println();

  last_error = error;

  pwm_pos = -(pwm_out);

  if(pwm_pos < 0)
  {
    pwm_pos = 0;
  }

  Serial.println(pwm_pos);

  Mot_PWM = pwm_pos * 10;

  if(Mot_PWM > 255)
  {
    Mot_PWM = 255;
  }
  else if (Mot_PWM <50 && Mot_PWM != 0)
  {
    analogWrite(fan1, 100);
    delay(1000);  
    Mot_PWM = 30;
  }

  //Mot_PWM = 70;
  analogWrite(fan1, Mot_PWM);  
}

