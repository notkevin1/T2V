/* v1.4 enables Bluetooth connectivity, Infrared object detection, and Line Following */

//======================================================
// Ultrasonic check obstacle and avoid Experiment
//===============================================================
int Echo = A1;  // Set Echo port
int Trig =A0;  //  Set Trig port 
int Distance = 0;

//=====================================================
//  Line Following Experiment
//===============================================================
//#include <Servo.h>
int Left_motor_back = 9;
int Left_motor_go = 8;
int Right_motor_go = 6;
int Right_motor_back = 7;
int Right_motor_en = 5;
int Left_motor_en = 10;

///*Set Button port*/
int key=13;
///*Set BUZZER port*/
int beep=12;

/*Set right &left LED port*/
int right_led=4;
int left_led=3;

/* Line Following */
const int SensorRight = A2;   	// Set Right Line Walking Infrared sensor port
const int SensorLeft = A3;     	// Set Left Line Walking Infrared sensor port
const int SensorLeftLeft = A5;  // Set Far Left Line Walking Infrared sensor port
int SL;    // State of Left Line Walking Infrared sensor
int SR;    // State of Right Line Walking Infrared sensor

/* Infrared obstacle avoidance */
const int SensorRight_2 = A4;     // Right  Infrared sensor
const int SensorLeft_2 = A5;     // Left  Infrared sensor
int SL_2;    // State of Left  Infrared sensor
int SR_2;    // State of Right  Infrared sensor

int redLight = 1;

//==============================
//Bluetooth protocol related
//==============================
int incomingByte = 0;       // Store Received Data(byte)
String inputString = "";         // Store Received Data(String)
boolean newLineReceived = false; // Previous data end flag
boolean startBit  = false;  // Protocol start flag
String returntemp = ""; // Store return data

void setup()
{
  //Initialize motor drive for output mode
  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_back, OUTPUT);
  pinMode(Right_motor_go, OUTPUT);
  pinMode(Right_motor_back, OUTPUT);
  pinMode(Right_motor_en, OUTPUT);
  pinMode(Left_motor_en, OUTPUT);
  pinMode(key,INPUT);// Set button as input
  pinMode(beep,OUTPUT);// Set buzzer as output

  pinMode(left_led, OUTPUT);//Set led as output
  pinMode(right_led, OUTPUT);//Set led as output
  pinMode(SensorRight, INPUT); // Set Right Line Walking Infrared sensor as input
  pinMode(SensorLeft, INPUT); // Set left Line Walking Infrared sensor as input
  pinMode(SensorLeftLeft, INPUT); // Set Far Left Line Walking Infrared sensor as input
//  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
//  digitalWrite(key,HIGH);//Initialize button
//  digitalWrite(beep,LOW);// set buzzer mute
  Serial.begin(9600); //  Set Bluetooth baud rate 9600
}

/*Format string initialization*/
int serial_putc( char c, struct __file * )
{
  Serial.write( c );
  return c;
}

/*Bluetooth receive*/
void Bluetooth(void)
{
  if (newLineReceived)
  {
    if (inputString[1] == '0')
    {
      brake();
      redLight = 1;
    }
    else if (inputString[1] = '1')
    {
      redLight = 0;
      run();
    }
    
    inputString = "";   // clear the string
    newLineReceived = false;

  }
}

void run()     // forward
{
  /*  TODO: Why digitalWrite HIGH/LOW and then analogWrite the pwm value?
      TODO: Adjust speed of car with pwm values */
  digitalWrite(Left_motor_en, HIGH); // Left motor enable
  digitalWrite(Right_motor_en, HIGH); // Right motor enable
  
//  analogWrite(Left_motor_en, 130); // commented to enable all four motors to spin. Enable can't be set to high 50% of the time?
//  analogWrite(Right_motor_en, 130);
  
  digitalWrite(Right_motor_go, HIGH); // right motor go ahead
  digitalWrite(Right_motor_back, LOW);
  digitalWrite(Left_motor_go, HIGH); // set left motor go ahead
  digitalWrite(Left_motor_back, LOW);
  
  analogWrite(Right_motor_go, 5); //PWM--Pulse Width Modulation(0~255). right motor go speed is 255.
  analogWrite(Right_motor_back, 0);
  analogWrite(Left_motor_go, 5); //PWM--Pulse Width Modulation(0~255).left motor go speed is 135.
  analogWrite(Left_motor_back, 0);
//  delay(time * 100);   //Running time can be adjusted
}

void brake()         //STOP
{
  /* TODO: Should we stick to all analogWrite? */ 
  digitalWrite(Right_motor_go, LOW); //Stop the right motor
  digitalWrite(Right_motor_back, LOW);
  digitalWrite(Left_motor_go, LOW); //Stop the left motor
  digitalWrite(Left_motor_back, LOW);
  //delay(time * 100);  //Running time can be adjusted
}

void left()        //turn left
{
  /*  TODO: Same questions on analogWrite/digitalWrite
      TODO: Adjsut turn speed with pwm value */

//  digitalWrite(Right_motor_go, HIGH);	// right motor go ahead
//  digitalWrite(Right_motor_back, LOW);
//  digitalWrite(Left_motor_go, LOW);  // left motor stop
//  digitalWrite(Left_motor_back, HIGH);

  digitalWrite(Left_motor_go,LOW);   // right motor stop
  digitalWrite(Left_motor_back,HIGH);
  analogWrite(Left_motor_go,0); 
  analogWrite(Left_motor_back,0);

  analogWrite(Right_motor_go, 190); // PWM--Pulse Width Modulation(0~255) control speed，right motor go speed is 255.
  analogWrite(Right_motor_back, 0);
  analogWrite(Left_motor_go, 0);
  analogWrite(Left_motor_back, 128); // 128 is the minimum value for Right_motor_back, see Right()
  
  /* turning left = left signal ON */
  digitalWrite(left_led, HIGH);
  digitalWrite(right_led, LOW);
  
  //delay(time * 100);
}

void spin_left(int time)   //Left rotation
{
  /*  TODO: Same questions on analogWrite/digitalWrite
      TODO: Adjsut turn speed with pwm value */
//  digitalWrite(Right_motor_go, HIGH); // right motor go ahead
//  digitalWrite(Right_motor_back, LOW);
  analogWrite(Right_motor_go, 200); // PWM--Pulse Width Modulation(0~255) control speed ,right motor go speed is 200.
  analogWrite(Right_motor_back, 0);
//  digitalWrite(Left_motor_go, LOW);  // left motor back off
//  digitalWrite(Left_motor_back, HIGH);
  analogWrite(Left_motor_go, 0);
  analogWrite(Left_motor_back, 200); // PWM--Pulse Width Modulation(0~255) control speed,left motor back speed is 200.
  delay(time * 100);
}

void right()      //turn right
{
  /* TODO: Same questions on analogWrite/digitalWrite
     TODO: Adjsut turn speed with pwm value, right turn set to turn faster than left turn */

//  digitalWrite(Left_motor_go, HIGH); // left motor go ahead
//  digitalWrite(Left_motor_back, LOW);
//  digitalWrite(Right_motor_go, LOW);  // right motor stop
//  digitalWrite(Right_motor_back, LOW);

  digitalWrite(Right_motor_go,LOW);   // right motor stop
  digitalWrite(Right_motor_back,HIGH);
  analogWrite(Right_motor_go,0); 
  analogWrite(Right_motor_back,0); 
  
  analogWrite(Right_motor_go, 0);
  analogWrite(Right_motor_back, 128); // 128 seems to be the minimum value in which the Right_motor_back can turn
  analogWrite(Left_motor_go, 190); // PWM--Pulse Width Modulation(0~255) control speed ,left motor go speed is 255.
  analogWrite(Left_motor_back, 0);


  /* turning right = right signal ON */
  digitalWrite(left_led, LOW);
  digitalWrite(right_led, HIGH);
  
  //delay(time * 100);
}

void spin_right(int time)   //Right rotation
{
  /*  TODO: Same questions on analogWrite/digitalWrite
      TODO: Adjsut turn speed with pwm value */
//  digitalWrite(Right_motor_go, LOW); // right motor back off
//  digitalWrite(Right_motor_back, HIGH);
  analogWrite(Right_motor_go, 0);
  analogWrite(Right_motor_back, 200); // PWM--Pulse Width Modulation(0~255) control speed
//  digitalWrite(Left_motor_go, HIGH); // left motor go ahead
//  digitalWrite(Left_motor_back, LOW);
  analogWrite(Left_motor_go, 200); // PWM--Pulse Width Modulation(0~255) control speed
  analogWrite(Left_motor_back, 0);
  delay(time * 100);
}

void back(int time)   //back off
{ 
//  digitalWrite(Right_motor_go, LOW); //right motor back off
//  digitalWrite(Right_motor_back, HIGH);
//  analogWrite(Right_motor_en, 165);
//  digitalWrite(Left_motor_go, LOW); //left motor back off
//  digitalWrite(Left_motor_back, HIGH);

  analogWrite(Right_motor_go, 0);
  analogWrite(Right_motor_back, 140); // PWM--Pulse Width Modulation(0~255) control speed
  analogWrite(Left_motor_go, 0);
  analogWrite(Left_motor_back, 140); // PWM--Pulse Width Modulation(0~255) control speed
  delay(time * 100);
}

void Distance_test()   // Measuring front distance
{
  digitalWrite(Trig, LOW);    // set trig port low level for 2μs
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  // set trig port high level for 10μs(at least 10μs)
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);    // set trig port low level
  float Fdistance = pulseIn(Echo, HIGH);  // Read echo port high level time(unit:μs)
  Fdistance= Fdistance/58;       // Distance(m) =(time(s) * 344(m/s)) / 2     /****** The speed of sound is 344m/s.*******/
                                 //  ==> 2*Distance(cm) = time(μs) * 0.0344(cm/μs)
                                 // ==> Distance(cm) = time(μs) * 0.0172 = time(μs) / 58
  Serial.print("Distance:");      //Output Distance(cm)
  Serial.println(Fdistance);         //display distance
  Distance = Fdistance;
}  

//==========================================================

//void keysacn()
//{
//  int val;
//  val=digitalRead(key);// Reads the button ,the level value assigns to val
//  while(digitalRead(key))// When the button is not pressed
//  {
//    val=digitalRead(key);
//  }
//  while(!digitalRead(key))// When the button is pressed
//  {
//    delay(10);	//delay 10ms
//    val=digitalRead(key);// Reads the button ,the level value assigns to val
//    if(val==LOW)  //Double check the button is pressed
//    {
//
//      digitalWrite(beep,HIGH);//The buzzer sounds
//      delay(50);//delay 50ms
//      while(!digitalRead(key))	//Determine if the button is released or not
//        digitalWrite(beep,LOW);//mute
//    }
//    else
//      digitalWrite(beep,LOW);//mute
//  }
//}

/*main loop*/
void loop()
{

  //  keysacn();//Press the button to start
  while (1)
  {
    while (Serial.available())
    {
      incomingByte = Serial.read();   //One byte by one byte reads 
      if (incomingByte == '$')  // '$' means the start of packet
      {
        startBit = true;
      }
      if (startBit == true)
      {
        inputString += (char) incomingByte;     // The received data constitutes a completed packet.
      }
      if (incomingByte == '#')    // '#' means the end of packet
      {
        newLineReceived = true;
        startBit = false;
      }
    }
    Bluetooth();
//    /**************************************************************************************
//      Infrared signal back means white undersurface ,returns low level and led lights up.
//      Infrared signal gone means black undersurface ,returns high level and led lights off.
//    **************************************************************************************/
    if (redLight == 0)
    {
      SR = digitalRead(SensorRight);//Right Line Walking Infrared sensor against white undersurface,then LED[L2] light illuminates and while against black undersurface,LED[L2] goes off
      SL = digitalRead(SensorLeft);//Left Line Walking Infrared sensor against white undersurface,then LED[L3] light illuminates and while against black undersurface,LED[L3] goes off
      SR_2 = digitalRead(SensorRight_2);//Right infrared sensor detects the obstacle,then LED[L5] light illuminates and otherwise it goes off.
      SL_2 = digitalRead(SensorLeft_2);//Left infrared sensor detects the obstacle,then LED[L4] light illuminates and otherwise it goes off.
  //    SLL = digitalRead(SensorLeftLeft);
      if (SL == LOW && SR == LOW && SL_2 == HIGH && SR_2 == HIGH) // Black lines were not detected at the same time
        run();   // go ahead
  //    else if (SLL == HIGH & SL == LOW & SR == HIGH)
  //      digitalWrite(beep,HIGH);
  //      spin_left(50);
      else if (SL == LOW & SR == HIGH)// Left sensor against white undersurface and right against black undersurface , the car left off track and need to adjust to the right.
        right();
      else if (SR == LOW & SL ==  HIGH) // Rihgt sensor against white undersurface and left against black undersurface , the car right off track and need to adjust to the left.
        left();
      else if (SL_2 == LOW & SR_2 == LOW) // Black lines were detected at the same time , the car stop.
        brake();
      else // Black lines were detected at the same time , the car stop.
        brake();
  //      delay(10*100);
    }
  }
}
