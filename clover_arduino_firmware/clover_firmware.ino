#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <PinChangeInterrupt.h>
#include <math.h>

// Pins
#define pin_left_a 12
#define pin_left_b 13
#define pin_right_a 6
#define pin_right_b 5
#define disable_pin_left 7
#define disable_pin_right 4
#define left_Forward 9
#define left_Backward 10
#define right_Forward 3
#define right_Backward 11

float x;
float y;
float z;
volatile long left_ticks = 0;
volatile long right_ticks = 0;
volatile float tmp_left = 0;
volatile float tmp_right = 0;

double elapsed_time;
unsigned long t_now;
unsigned long t_prev = 0; 

double error_left = 0;
double error_right = 0;

double error_cum_left = 0;
double error_cum_right = 0;

int K = 340; //(2*pi)e6/18800 (ticks per turn)
double kp = 50;
double ki = 0.02e-2; //being time computed in microseconds, this value should be very small

ros::NodeHandle nh;
std_msgs::Float32 left_wheel_speed;
std_msgs::Float32 right_wheel_speed;


void velCallback(  const geometry_msgs::Twist& vel) {

      
     x = vel.linear.x; // I CAN USE VEL AS I WANT
     y = vel.linear.y;
     z = vel.angular.z;

     x = x/3.33;
     z = z/2;   
    }


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCallback);
ros::Publisher pub_l("left_speed", &left_wheel_speed);
ros::Publisher pub_r("right_speed", &right_wheel_speed);


void setup() {
  
    pinMode(left_Forward, OUTPUT);
    pinMode(left_Backward, OUTPUT);
    pinMode(right_Forward, OUTPUT);
    pinMode(right_Backward, OUTPUT);
    pinMode(disable_pin_left, OUTPUT);
    pinMode(disable_pin_right, OUTPUT);
    pinMode(pin_left_a, INPUT);
    pinMode(pin_left_b, INPUT);
    pinMode(pin_right_a, INPUT);
    pinMode(pin_right_b, INPUT);

//    //Set the pins mode through registers (1-->OUTPUT, 0-->INPUT)
//    DDRB = B0011111;  //PORT B: pins(13,..,8)
//    DDRD = DDRD|B1001100;  //PORT D: pins(0,..,7)
//
//    //set timer0 interrupt at 2kHz
//    TCCR0A = 0;// set entire TCCR0A register to 0
//    TCCR0B = 0;// same for TCCR0B
//    TCNT0  = 0;//initialize counter value to 0
//    
//    // set compare match register for 2khz increments
//    OCR0A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
//    TCCR0A |= (1 << WGM01); // turn on CTC mode
//    TCCR0B |= (1 << CS01) | (1 << CS00);  // Set CS01 and CS00 bits for 64 prescaler     
//    TIMSK0 |= (1 << OCIE0A);  // enable timer compare interrupt
//  
//    sei(); //allow interrupts           
//    
    // settings for TIMER 1: pins 9, 10
    TCCR1A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
    TCCR1B = _BV(CS00);

    // settings for TIMER 2: pins 3, 11
    TCCR2A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
    TCCR2B = _BV(CS00);

    

    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pub_l);
    nh.advertise(pub_r);   

    Serial.begin(57600);
    
    digitalWrite(disable_pin_left, LOW);
    digitalWrite(disable_pin_right, LOW); 

    // attach the new PinChangeInterrupts and enable event functions below
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(pin_left_a), left_ticks_a, RISING);
//    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(pin_left_b), left_ticks_b, RISING);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(pin_right_a), right_ticks_a, RISING);
//    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(pin_right_b), right_ticks_b, RISING);
}

void loop() {
      ////////// compute wheels speed from encoder readings //////////////
  
      static long left_ticks_prev = 0;
      static long right_ticks_prev = 0;
      static long time_prev = 0;
      long int time_now = micros();
      
      tmp_left = K*((float)(left_ticks - left_ticks_prev)/(time_now - time_prev));
      tmp_right = K*((float)(right_ticks - right_ticks_prev)/(time_now - time_prev));
      left_wheel_speed.data = tmp_left;
      right_wheel_speed.data = tmp_right;
      left_ticks_prev = left_ticks;
      right_ticks_prev = right_ticks;
      time_prev = time_now;   
      
      // publish the topics
      pub_l.publish(&left_wheel_speed);
      pub_r.publish(&right_wheel_speed);
      nh.spinOnce();

      /////////// set motor speed ////////////////////
      double u_left;
      double u_right;
 
      double left_vel = (x - z);  
      double right_vel = (x + z);
      
      // with max PWM value = 230 (90% of 255), 4.8 rad/s. Maximum linear speed = 0.48 m/s 
      if (left_vel>0.45)    //impone max wheels speed: 0.4 m/s --> 4 rad/s
       left_vel = 0.45;
      if (right_vel>0.45)
       right_vel = 0.45;
      if (left_vel<-0.45)
       left_vel = -0.45;
      if (right_vel<-0.45)
       right_vel = -0.45;  
      
      t_now = micros();
      elapsed_time = (double)(t_now - t_prev); 
      
      //Compute error values
      error_left = left_vel/0.1-left_wheel_speed.data;
      error_right = right_vel/0.1-right_wheel_speed.data;    

     
      //Compute comulated error values
      error_cum_left += error_left*elapsed_time; 
      error_cum_right += error_right*elapsed_time;
     
      u_left = kp*error_left + ki*error_cum_left;
      u_right = kp*error_right + ki*error_cum_right;

     // add control variable saturation
     if(u_left>230)
      u_left = 230;
     if(u_right>230)
      u_right = 230;
     if(u_left<-230)
      u_left = -230;
     if(u_right<-230)
      u_right = -230;
   
     t_prev = t_now;    
          
     // PWM

     if (u_left > 0){
          OCR1A = 255-(int)u_left;   // 230/0.48 = 480 --> 0.4*480 = 192 (75% absolute max pwm, 83% max raccomended pwm)
          OCR1B = 255;
          }
     else if(u_left < 0){
          OCR1A = 255;
          OCR1B = 255-(int)abs(u_left);
          }
     else{
          OCR2A = 255;
          OCR2B = 255;
          }           
     if (u_right > 0){
          OCR2A = 255-(int)u_right;   // 230/0.48 = 480 --> 0.4*480 = 192 (75% absolute max pwm, 83% max raccomended pwm)
          OCR2B = 255;
          }
     else if(u_right <= 0){
          OCR2A = 255;
          OCR2B = 255-(int)abs(u_right);
          }                
     else{
          OCR2A = 255;
          OCR2B = 255;
          }        
     nh.spinOnce();
     delay(5); //200 Hz
}


///////////////////////////////////////////////////////
////// INTERRUPTS SUBROUTINES DEFINITION //////////////
///////////////////////////////////////////////////////


void left_ticks_a(void) {
  if((PINB & B00100000)==32)  //ADD EXPLANATION FOR HUMANS
    left_ticks--;  // increase value
  else
    left_ticks++;  // decrease value
}
//
//void left_ticks_b(void) {
//  if((PINB & B00010000)==16)
//    left_ticks++;  // increase value
//  else
//    left_ticks--;  // decrease value
//}

void right_ticks_a(void) {
  if((PIND & B00100000)==32)
    right_ticks--;  // increase value
  else
    right_ticks++;  // decrease value
}

//void right_ticks_b(void) {
//  if((PIND & B01000000)==64)
//    right_ticks++;  // increase value
//  else
//    right_ticks--;  // decrease value
//}
