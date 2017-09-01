#include <SparkFun_Si7021_Breakout_Library.h>
#include <Wire.h>
//#include <PID_v1.h>


//PID Controller Variables
/*float Kp = 1;
float Ki = 10;
float Kd = 1;*/

float kp = 1;
float ki = 1;
float kd = 1;

unsigned long timeVal = 5000;
float target_position = 23; 

/*double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
const int sampleRate = 10;
const long serialPing =500;
unsigned long now = 0;
unsigned long lastMessage = 0;*/


float humidity = 0;
float tempC = 0;

int fan1 = 3;   //Exhaust Fan
int fan2 = 5;   //Stimulation Fan
int fan3 = 6;   //Input Fan

Weather sensor;

//Function prototypes
//void tempVal();
void PID_init();
void PID_Control();


//Custom PID Code Variables
float current_position = 0;
float error = 0;
float integral = 0;
float derivative = 0;
float pwm_out = 0;

float last_error = 0;
int x = 0;

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

/*void tempVal(){
  humidity = sensor.getRH();  //Triggers reading temperature and humidity
  tempC = sensor.getTemp();   //Reading temperature value from last humidity reading
}*/


/*void PID_init()
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
}*/

void PID_Loop()
{
  float pwm_pos = 0;
  int Mot_PWM = 0;
  current_position = sensor.getTemp();

  error = target_position - current_position;

  integral = integral + error;

  if(integral > 255)
  {
    integral = 255;
  }

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

