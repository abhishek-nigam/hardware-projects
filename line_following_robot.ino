////////IR sensors/////////////// 
#define ir_1_vcc 33 
#define ir_1_gnd 35 
#define ir_1_out 31

#define ir_2_vcc 39 
#define ir_2_gnd 41
#define ir_2_out 37 

#define ir_3_vcc 45 
#define ir_3_gnd 43 
#define ir_3_out 47 

#define ir_4_vcc 49
#define ir_4_gnd 51
#define ir_4_out 53

#define ir_5_vcc 50
#define ir_5_gnd 52
#define ir_5_out 48

#define ir_6_vcc 44
#define ir_6_gnd 46
#define ir_6_out 42

///////////Motor control////////////
#define mtr_left_1 9
#define mtr_left_2 8

#define mtr_right_1 11
#define mtr_right_2 10

//Other variables
//Kp = 5, Ki = 0, Kd = 10 -->good
//Kp = 2, Ki = 0, Kd = 0 -->initial
//Kp = 5, Ki = 0, Kd = 30 -->check

float Kp = 5, Ki = 0, Kd = 30;
float P = 0, I = 0, D = 0, PID_value = 0;
float error = 0, prev_error = 0, prev_I = 0;

int initial_motor_speed = 255;
int sensor[6] = {0};
unsigned long currentTime = 0, prevTime = 0;

bool rightFlag = false, leftFlag = false, elseExecuted = false;

void readSensor(void);
void calculatePID(void);
void motorControl(void);
int checkFront(int);

void setup() {
  // IR sensor initiasaitoon
  pinMode(ir_1_vcc,OUTPUT);
  pinMode(ir_1_gnd,OUTPUT);
  pinMode(ir_1_out,INPUT);

  pinMode(ir_2_vcc,OUTPUT);
  pinMode(ir_2_gnd,OUTPUT);
  pinMode(ir_2_out,INPUT);

  pinMode(ir_3_vcc,OUTPUT);
  pinMode(ir_3_gnd,OUTPUT);
  pinMode(ir_3_out,INPUT);

  pinMode(ir_4_vcc,OUTPUT);
  pinMode(ir_4_gnd,OUTPUT);
  pinMode(ir_4_out,INPUT);

  pinMode(ir_5_vcc,OUTPUT);
  pinMode(ir_5_gnd,OUTPUT);
  pinMode(ir_5_out,INPUT);

  pinMode(ir_6_vcc,OUTPUT);
  pinMode(ir_6_gnd,OUTPUT);
  pinMode(ir_6_out,INPUT);

  digitalWrite(ir_1_vcc,HIGH);
  digitalWrite(ir_1_gnd,LOW);

  digitalWrite(ir_2_vcc,HIGH);
  digitalWrite(ir_2_gnd,LOW);

  digitalWrite(ir_3_vcc,HIGH);
  digitalWrite(ir_3_gnd,LOW);

  digitalWrite(ir_4_vcc,HIGH);
  digitalWrite(ir_4_gnd,LOW);

  digitalWrite(ir_5_vcc,HIGH);
  digitalWrite(ir_5_gnd,LOW);

  digitalWrite(ir_6_vcc,HIGH);
  digitalWrite(ir_6_gnd,LOW);
  
    //Motor initialisation
  pinMode(mtr_left_1,OUTPUT);
  pinMode(mtr_left_2,OUTPUT);
  pinMode(mtr_right_1,OUTPUT);
  pinMode(mtr_right_2,OUTPUT);
  
  Serial.begin(9600);

}

void loop() {

  readSensor();
  calculatePID();
  motorControl();
}

int checkFront(int n)
{
  if(sensor[5]==0)
  {
    return 0;
  }
  else
  {
    return n;
  }
}

void readSensor()
{
  sensor[0] = digitalRead(ir_1_out);
  sensor[1] = digitalRead(ir_2_out);
  sensor[2] = digitalRead(ir_3_out);
  sensor[3] = digitalRead(ir_4_out);
  sensor[4] = digitalRead(ir_5_out);
  sensor[5] = digitalRead(ir_6_out);
/*
  Serial.print(sensor[0]);
  Serial.print(" ");
  Serial.print(sensor[1]);
  Serial.print(" ");
  Serial.print(sensor[2]);
  Serial.print(" ");
  Serial.print(sensor[3]);
  Serial.print(" ");
  Serial.print(sensor[4]);
  Serial.print(" ");
  Serial.println(sensor[5]);
*/
  elseExecuted = false;
  
  //Bot is right of track, hence it should turn left
  if(sensor[0]==0 && sensor[1]==1 && sensor[2]==1 && sensor[3]==1 && sensor[4]==1) //01111
    error = 3;
  else if(sensor[0]==0 && sensor[1]==0 && sensor[2]==1 && sensor[3]==1 && sensor[4]==1) //00111
    error = checkFront(2);
  else if(sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==1 && sensor[4]==1) //00011 -Left 90deg turn 
    {
      error = checkFront(1);
      /*
      if(error == 1)
      {
       currentTime = millis(); 
       if((currentTime - prevTime)>1000)
       {
        leftFlag = true; 
       }
       prevTime = currentTime;
      }*/
    }
  else if(sensor[0]==1 && sensor[1]==0 && sensor[2]==0 && sensor[3]==1 && sensor[4]==1) //10011
    error = checkFront(1);
  else if(sensor[0]==1 && sensor[1]==0 && sensor[2]==1 && sensor[3]==1 && sensor[4]==1) //10111
    error = 1;
  
  //Bot is in correct position, hence continue forward  
  else if(sensor[0]==1 && sensor[1]==1 && sensor[2]==0 && sensor[3]==1 && sensor[4]==1) //11011
    {
      error = 0;
      /*
      if(leftFlag == true)
      {
        leftFlag = false;
        //Serial.print("L");
      }
      else if(rightFlag == true)
      {
        rightFlag = false;
        //Serial.print("R");
      }*/
    }
    
  //Bot is left of track, hene it should turn right
  else if(sensor[0]==1 && sensor[1]==1 && sensor[2]==0 && sensor[3]==0 && sensor[4]==1) //11001
   { error = checkFront(-1);  }
  else if(sensor[0]==1 && sensor[1]==1 && sensor[2]==1 && sensor[3]==0 && sensor[4]==1) //11101
    error = -1;
  else if(sensor[0]==1 && sensor[1]==1 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0) //11000  -Right 90deg turn
    {
      
      error = checkFront(-1);
      /*
      if(error == -1)
      {
       currentTime = millis(); 
       if((currentTime - prevTime)>1000)
       {
        rightFlag = true; 
       }
       prevTime = currentTime;
      }*/
    }
  else if(sensor[0]==1 && sensor[1]==1 && sensor[2]==0 && sensor[3]==0 && sensor[4]==1) //11100
    error = checkFront(-2);
  else if(sensor[0]==1 && sensor[1]==1 && sensor[2]==1 && sensor[3]==1 && sensor[4]==0) //11110
    error = -3;



  /////////////Special cases --Y-intersection//////////////////////////
 // TODO: ?Add if to all Y statements that sensor[5]==1
  
  //Bot in middle of Y
  else if(sensor[0]==1 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==1) //10001
    error = 2;
  else if(sensor[0]==1 && sensor[1]==0 && sensor[2]==1 && sensor[3]==0 && sensor[4]==1) //10101
    error = 2;
  else if(sensor[0]==0 && sensor[1]==1 && sensor[2]==1 && sensor[3]==1 && sensor[4]==0) //01110
    error = 3;

  //Bot slightly right
  else if(sensor[0]==0 && sensor[1]==1 && sensor[2]==0 && sensor[3]==1 && sensor[4]==1) //01011
    error = 1;
  else if(sensor[0]==0 && sensor[1]==1 && sensor[2]==1 && sensor[3]==0 && sensor[4]==1) //01101
    error = 2;

  
  //Bot slightly left
  else if(sensor[0]==1 && sensor[1]==1 && sensor[2]==0 && sensor[3]==1 && sensor[4]==1) //11010
    error = -1;
  else if(sensor[0]==1 && sensor[1]==0 && sensor[2]==1 && sensor[3]==1 && sensor[4]==0) //10110
    error = -2;
  //////////////////////Y-cases end///////////////////////////////////

  
  //Extreme error cases, hence should check previous_error  
  else if(sensor[0]==1 && sensor[1]==1 && sensor[2]==1 && sensor[3]==1 && sensor[4]==1) //11111
  { if(prev_error==3)
    {error = 4; }
    else if(prev_error==-3)
    {error = -4; }
  }

 else if(sensor[0]==0 && sensor[1]==0 && sensor[2]==0 && sensor[3]==0 && sensor[4]==0)//00000 - T or + intersection --> take left turn
 {
  
    error = checkFront(1);
    
    /*
    if((error != 0)&&(millis()>1000)) //finish condition, stop all motors
    {
     digitalWrite(mtr_left_1,LOW);
     digitalWrite(mtr_left_2,LOW);
     digitalWrite(mtr_right_1,LOW);
     digitalWrite(mtr_right_2,LOW);
     exit(0); 
    }
    */
 }
 else
 {
    elseExecuted = true;
    Serial.println("Else executed");
 }
 
}

//10000,00001,00000

void calculatePID()
{
  P = error;
  I = I + error;
  D = error - prev_error;

  PID_value = (Kp*P) + (Ki*I) + (Kd*D);
  
  prev_error = error;
}

void motorControl()
{
      int left_motor_speed;
      int right_motor_speed;

      if(elseExecuted == false)
      {
        left_motor_speed = initial_motor_speed + PID_value;
        right_motor_speed = initial_motor_speed - PID_value;  
      }
      else
      {
        left_motor_speed = 0.7*initial_motor_speed;
        right_motor_speed = 0.7*initial_motor_speed;
      }
         

      
      if(left_motor_speed > 0)
      {
        constrain(left_motor_speed,0,255);
        analogWrite(mtr_left_1,left_motor_speed);
        analogWrite(mtr_left_2,0);
      }
      else
      {
        left_motor_speed = -left_motor_speed;
        constrain(left_motor_speed,0,255);
        analogWrite(mtr_left_1,0);
        analogWrite(mtr_left_2,left_motor_speed);    
      }
    
      if(right_motor_speed > 0)
      {
        constrain(right_motor_speed,0,255);
        analogWrite(mtr_right_1,right_motor_speed);
        analogWrite(mtr_right_2,0);
      }
      else
      {
        right_motor_speed = -right_motor_speed;
        constrain(right_motor_speed,0,255);
        analogWrite(mtr_right_1,0);
        analogWrite(mtr_right_2,right_motor_speed);    
      }  
  
}

