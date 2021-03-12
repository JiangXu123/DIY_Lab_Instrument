#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ClickEncoder.h>
#include <TimerOne.h>
#include <Bounce2.h>
Bounce debouncer = Bounce(); 

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Declare pin functions on Redboard
#define stp1 5
#define dir1 6
#define MS11 7
#define MS21 8
#define EN1  9

#define stp2 22
#define dir2 23
#define MS12 24
#define MS22 25
#define EN2  26

#define stp3 27
#define dir3 28
#define MS13 29
#define MS23 30
#define EN3  31

#define stp4 32     //this is temporaly assigned to pump
#define dir4 34
#define MS14 36
#define MS24 38
#define EN4  40

#define stp5 33     //this is temporaly assigned to thita axis, which is not present to current setup, just used to disable the chip 
#define dir5 35
#define MS15 37
#define MS25 39
#define EN5  41

#define X_pin A0
#define Y_pin A1
#define Joystick_clk 2
#define Coordinate_clk 18
#define Shuttle_clk 19

#define left 48
#define right 45
#define up 44
#define down 49
#define right_turn 46
#define left_turn 47
#define pump_asp 12
#define pump_blow 11
#define L293D_Enable1 51
#define L293D_Input1 52
#define L293D_Input2 53
#define LED_switch 42
#define LED_output 43

ClickEncoder *encoder;
int16_t last, value, Pump_value; // these variables are for the click encoder
float Volume; // Pump volume is expressed in increment of 0.1 ul if 250 ul microinjector is used.  

struct PositionStruct{
  int xVal;
  int yVal;
  int zVal;
};
PositionStruct arrayofPositions[20];
PositionStruct relativePostions[20];

volatile int8_t CLICK_COUNT = 1; //These variables are to monitor the click of the joy stick;
int8_t S_count = 0; //S_count is to used to discremenate between the working docking zone and the the initial site, helping the machine to determine the shuttle direction 
volatile int8_t pos_mem_count = 1;//pos_mem_count is used to number the key position information when the tip move from the intial site to the work docking site. 
volatile int8_t pump_state_count = 0;//pump_state_count is used to memorise the state of the pump in order we can use it to control whether or not to move the pump for a fixed volume or not.

int X_num;
int Y_num;
int X_position = 0;
int Y_position = 0;

int steptime1;      //These variables are to store the speed of the stepper motor of the 3 axis.
int steptime2;
int steptime3;

int x = 0; //set the coordinate variable and the initial coordinate to (0,0,0)
int y = 0;
int z = 0;

int a = 0; //These are the new directional positions that are generated after the tip is moved from the work docking point. 
int b = 0;
int c = 0;

int sl_mv_delay = 40;
int pump_count = 2425;  //This is the  pump movement count 250 ul equals to 2425 full steps of the stepper motor.

int Speed1 = 10;    //Speed1 is the L239D on time
int Speed2 = 30;   //Speed2 is the L239D off time
void timerIsr() {
  encoder->service();
}
int left_switch;
int right_switch;
int up_switch;
int down_switch;

int pump_switch1;
int pump_swithc2;
int right_turn_sw;      //when this button is on, capilary turn right 
int left_turn_sw;       //when this button is on, capilary turn left
int LED_state = 1;
int LED_switch_state;

void setup() {
  Serial.begin(9600);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  pinMode (Joystick_clk, INPUT_PULLUP);
  pinMode (Coordinate_clk,INPUT_PULLUP);
  
  debouncer.attach(Shuttle_clk,INPUT_PULLUP); // Attach the debouncer to Shuttle_clk with INPUT_PULLUP mode
  debouncer.interval(25); // Use a debounce interval of 25 milliseconds
  
  attachInterrupt(0, Joystick_Click, RISING);
  attachInterrupt(5, coordinate_mem, RISING);
  //attachInterrupt(4, shuttle_count, RISING);
  pinMode(stp1, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(MS11, OUTPUT);
  pinMode(MS21, OUTPUT);
  pinMode(EN1, OUTPUT);

  pinMode(stp2, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(MS12, OUTPUT);
  pinMode(MS22, OUTPUT);
  pinMode(EN2, OUTPUT);

  pinMode(stp3, OUTPUT);
  pinMode(dir3, OUTPUT);
  pinMode(MS13, OUTPUT);
  pinMode(MS23, OUTPUT);
  pinMode(EN3, OUTPUT);

  pinMode(stp4, OUTPUT);
  pinMode(dir4, OUTPUT);
  pinMode(MS14, OUTPUT);
  pinMode(MS24, OUTPUT);
  pinMode(EN4, OUTPUT);

  pinMode(stp5, OUTPUT);
  pinMode(dir5, OUTPUT);
  pinMode(MS15, OUTPUT);
  pinMode(MS25, OUTPUT);
  pinMode(EN5, OUTPUT);

  pinMode(left, INPUT_PULLUP);
  pinMode(right, INPUT_PULLUP);
  pinMode(up, INPUT_PULLUP);
  pinMode(down, INPUT_PULLUP);
  pinMode(right_turn, INPUT_PULLUP);
  pinMode(left_turn, INPUT_PULLUP);
  pinMode(pump_asp, INPUT_PULLUP);
  pinMode(pump_blow, INPUT_PULLUP);
  pinMode(L293D_Enable1, INPUT);
  pinMode(L293D_Input1, INPUT);
  pinMode(L293D_Input2,INPUT);
  pinMode(LED_output, OUTPUT);
  pinMode(LED_switch, INPUT_PULLUP);
 
  resetEDPins();                      //let all the driver to rest.
  arrayofPositions[0].xVal = 0;       //the initial position is set to {0,0,0} by default.
  arrayofPositions[0].yVal = 0;
  arrayofPositions[0].zVal = 0;
  
  encoder = new ClickEncoder(A2, A3, A4);  //click pin was connected to pin A4
  Timer1.initialize(1000); // Timer1 will zero the count ever 1000 microsecond(1 mS)
  Timer1.attachInterrupt(timerIsr); 
  last = 0;
}

void Joystick_Click() {
  CLICK_COUNT++;
  if(CLICK_COUNT == 3){ //each click will toggle CLICK_COUNT between 1 and 2
    CLICK_COUNT = 1;}
}

void shuttle_count(){
    S_count++;
 }

 void coordinate_mem(){
 arrayofPositions[pos_mem_count].xVal = x;
 arrayofPositions[pos_mem_count].yVal = y;
 arrayofPositions[pos_mem_count].zVal = z;
 pos_mem_count++;
}

void pump_state(){
  pump_state_count++;
}
 
void loop() {
  X_num = analogRead(X_pin);
  Y_num = analogRead(Y_pin);
  X_position = X_num - 512;
  Y_position = Y_num - 512;
  steptime1 = int(-0.0957 * abs(X_position) + 50.3);//float f = .....;int i = (int)(f + 0.5);i就是rounded up of f(四舍五入的结果)
  steptime2 = int(-0.0957 * abs(Y_position) + 50.3);
  steptime3 = int(-0.0957 * abs(Y_position) + 50.3);
  
  debouncer.update(); // Update the Bounce instance
  if ( debouncer.fell() ) {  // Call code if button transitions from HIGH to LOW
   S_count++;
   }
   
  value += encoder->getValue();  // This part is to read the value generated from turning of the rotary encoder. 
  if (value != last) {
    last = value;
    Pump_value = value/4;  //one "click turn"  of the encoder will cause increment of 'value' by 4. Pump_value is the true full numbers of steps of the pump stepper motor. For 250 ul microinjector it is equal to around 0.1 ul(250 ul/2425 step) 
    Volume = 0.1 * Pump_value;
    }
  
  ClickEncoder::Button b = encoder->getButton();   //This part is to monitor the click of the click encoder. 
  if (b != ClickEncoder::Open) {
    switch (b) {
      case ClickEncoder::DoubleClicked:
          //Serial.println("ClickEncoder::DoubleClicked");
          encoder->setAccelerationEnabled(!encoder->getAccelerationEnabled());
          //Serial.print("  Acceleration is ");
          //Serial.println((encoder->getAccelerationEnabled()) ? "enabled" : "disabled");
      case ClickEncoder::Clicked:
          pump_state(); // single click of the rotary encoder will cause the movement of the pump by 'Pump_value' steps
        break;
    }
  }
    
  if (X_position > 5 && CLICK_COUNT == 1)
  {
    SmallStepForwardMode1();
  }
  
  if (X_position < -5 && CLICK_COUNT == 1)
  {
    SmallStepBackwardMode1();
  }

  if (Y_position > 5 && CLICK_COUNT == 1)
  {
    SmallStepForwardMode2();
  }

  if (Y_position < -5 && CLICK_COUNT == 1)
  {
    SmallStepBackwardMode2();
  }

  if (Y_position > 5 && CLICK_COUNT == 2)
  {
    SmallStepForwardMode3();
  }

  if (Y_position < -5 && CLICK_COUNT == 2)
  {
    SmallStepBackwardMode3();
  }

  
  if (Y_position > 5 && CLICK_COUNT == 3)
  {
    SmallStepForwardMode4();
  }

  if (Y_position < -5 && CLICK_COUNT == 3)
  {
    SmallStepBackwardMode4();
  }
  if (Y_position > 5 && CLICK_COUNT == 4)
  {
    SmallStepForwardMode5();
  }

  if (Y_position < -5 && CLICK_COUNT == 4)
  {
    SmallStepBackwardMode5();
  }
  
  resetEDPins();
  if (abs(X_position) < 5 && abs(Y_position) < 5){
  print_coordinate();
  }
  
  if(S_count == 1 || S_count == 3){
    shuttle();}
  if(pump_state_count == 1){
    Pump_move();
    }
LED_switch_state = digitalRead(LED_switch);
if(LED_switch_state == LOW){
  delay(50);
  if(LED_switch_state == LOW){
    LED_state++;
    if(LED_state == 3){
      LED_state = 1;}
    }
    }
if(LED_state == 2){
  digitalWrite(LED_output, HIGH);
  }
if(LED_state == 1){
  digitalWrite(LED_output, LOW);
  }
left_switch = digitalRead(left);
right_switch = digitalRead(right);
up_switch = digitalRead(up);
down_switch = digitalRead(down);
  while(digitalRead(left) == LOW){
    left_switch = digitalRead(left);
    slow_move_left();
    }
  while(digitalRead(right) == LOW){
    right_switch = digitalRead(right);
    slow_move_right();
    }
  while(digitalRead(down) == LOW){
    up_switch = digitalRead(up);
    slow_move_up();
    }
  while(digitalRead(up) == LOW){
    down_switch = digitalRead(down);
    slow_move_down();
    }
  
  right_turn_sw = digitalRead(right_turn);
  left_turn_sw = digitalRead(left_turn);
  
  while(left_turn_sw == LOW){   // these codes are used to control the L293D to control the turn of the geared DC motor, which is used to control the turn of the capiliary.
   left_turn_sw = digitalRead(left_turn);
    digitalWrite(L293D_Enable1, HIGH);
    digitalWrite(L293D_Input1, HIGH);
    digitalWrite(L293D_Input2, LOW);
    delayMicroseconds(Speed1);
    digitalWrite(L293D_Input1, LOW);
    digitalWrite(L293D_Input2, LOW);
    delayMicroseconds(Speed2);                     // the ratio between Speed1 and Speed2 controls the rotation speed of the DC motro
    }
      
  while(right_turn_sw == LOW){
    right_turn_sw = digitalRead(right_turn);
    digitalWrite(L293D_Enable1, HIGH);
    digitalWrite(L293D_Input1,LOW);
    digitalWrite(L293D_Input2,HIGH);
    delayMicroseconds(Speed1);
    digitalWrite(L293D_Input1, LOW);
    digitalWrite(L293D_Input2, LOW);
    delayMicroseconds(Speed2); 
    }
    int pump_switch1 = digitalRead(pump_asp);
    int pump_switch2 = digitalRead(pump_blow);
    
  while(pump_switch1 == LOW){
    pump_switch1 = digitalRead(pump_asp);
    pump_Asp();
    }
  while(pump_switch2 == LOW){
    pump_switch2 =digitalRead(pump_blow);
    pump_Blow();
    }
}

void slow_move_left(){
  digitalWrite(EN2, LOW);
  digitalWrite(dir2, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS12, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  digitalWrite(MS22, HIGH);
  digitalWrite(stp2, HIGH); //Trigger one step
  delay(sl_mv_delay);
  digitalWrite(stp2, LOW); //Pull step pin low so it can be triggered again
  delay(sl_mv_delay);
  y++;
  }
  
void slow_move_right(){
  digitalWrite(EN2, LOW);
  digitalWrite(dir2, HIGH); //Pull direction pin low to move "forward"
  digitalWrite(MS12, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  digitalWrite(MS22, HIGH);
  digitalWrite(stp2, HIGH); //Trigger one step
  delay(sl_mv_delay);
  digitalWrite(stp2, LOW); //Pull step pin low so it can be triggered again
  delay(sl_mv_delay);
  y--;}
  
void slow_move_up(){
  digitalWrite(EN1, LOW);
  digitalWrite(dir1, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS11, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  digitalWrite(MS21, HIGH);
  digitalWrite(stp1, HIGH); //Trigger one step
  delay(sl_mv_delay);
  digitalWrite(stp1, LOW); //Pull step pin low so it can be triggered again
  delay(sl_mv_delay);
  x++;  
  }
  
void slow_move_down(){  
  digitalWrite(EN1, LOW);
  digitalWrite(dir1, HIGH); //Pull direction pin low to move "forward"
  digitalWrite(MS11, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  digitalWrite(MS21, HIGH);
  digitalWrite(stp1, HIGH); //Trigger one step
  delay(sl_mv_delay);
  digitalWrite(stp1, LOW); //Pull step pin low so it can be triggered again
  delay(sl_mv_delay);
  x--;}

void pump_Asp(){
  digitalWrite(EN4, LOW);
  digitalWrite(dir4, HIGH); //Pull direction pin low to move "forward"
  //digitalWrite(MS14, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  //digitalWrite(MS24, HIGH);
  digitalWrite(stp4, HIGH); //Trigger one step
  delay(1);
  digitalWrite(stp4, LOW); //Pull step pin low so it can be triggered again
  delay(1);
  pump_count++;
  }
  
void pump_Blow(){
  digitalWrite(EN4, LOW);
  digitalWrite(dir4, LOW); //Pull direction pin low to move "forward"
  //digitalWrite(MS14, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  //digitalWrite(MS24, HIGH);
  digitalWrite(stp4, HIGH); //Trigger one step
  delay(1);
  digitalWrite(stp4, LOW); //Pull step pin low so it can be triggered again
  delay(1);
  pump_count--;
  }

void SmallStepForwardMode1()
{
  digitalWrite(EN1, LOW);
  digitalWrite(dir1, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS11, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  digitalWrite(MS21, HIGH);
  digitalWrite(stp1, HIGH); //Trigger one step
  delay(steptime1);
  digitalWrite(stp1, LOW); //Pull step pin low so it can be triggered again
  delay(steptime1);
  x++;  
}

void SmallStepForwardMode2()
{
  digitalWrite(EN2, LOW);
  digitalWrite(dir2, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS12, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  digitalWrite(MS22, HIGH);
  digitalWrite(stp2, HIGH); //Trigger one step
  delay(steptime2);
  digitalWrite(stp2, LOW); //Pull step pin low so it can be triggered again
  delay(steptime2);
  y++;
}
void SmallStepForwardMode3()
{
  digitalWrite(EN3, LOW);
  digitalWrite(dir3, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS13, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  digitalWrite(MS23, HIGH);
  digitalWrite(stp3, HIGH); //Trigger one step
  delay(steptime3);
  digitalWrite(stp3, LOW); //Pull step pin low so it can be triggered again
  delay(steptime3);
  z++;
}

void SmallStepForwardMode4(){
  digitalWrite(EN4, LOW);
  digitalWrite(dir4, LOW); //Pull direction pin low to move "forward"
  //digitalWrite(MS14, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  //digitalWrite(MS24, HIGH);
  digitalWrite(stp4, HIGH); //Trigger one step
  delay(steptime2);
  digitalWrite(stp4, LOW); //Pull step pin low so it can be triggered again
  delay(steptime2);
  pump_count--;
  Serial.println(steptime2);
}
void SmallStepForwardMode5(){
  digitalWrite(EN5, LOW);
  digitalWrite(dir5, LOW); //Pull direction pin low to move "forward"
  //digitalWrite(MS14, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  //digitalWrite(MS24, HIGH);
  digitalWrite(stp5, HIGH); //Trigger one step
  delay(steptime2);
  digitalWrite(stp5, LOW); //Pull step pin low so it can be triggered again
  delay(steptime2);
}
void SmallStepBackwardMode1(){
  digitalWrite(EN1, LOW);
  digitalWrite(dir1, HIGH); //Pull direction pin low to move "forward"
  digitalWrite(MS11, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  digitalWrite(MS21, HIGH);
  digitalWrite(stp1, HIGH); //Trigger one step
  delay(steptime1);
  digitalWrite(stp1, LOW); //Pull step pin low so it can be triggered again
  delay(steptime1);
  x--;
}

void SmallStepBackwardMode2(){
  digitalWrite(EN2, LOW);
  digitalWrite(dir2, HIGH); //Pull direction pin low to move "forward"
  digitalWrite(MS12, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  digitalWrite(MS22, HIGH);
  digitalWrite(stp2, HIGH); //Trigger one step
  delay(steptime2);
  digitalWrite(stp2, LOW); //Pull step pin low so it can be triggered again
  delay(steptime2);
  y--;
}
void SmallStepBackwardMode3(){
  digitalWrite(EN3, LOW);
  digitalWrite(dir3, HIGH); //Pull direction pin low to move "forward"
  digitalWrite(MS13, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  digitalWrite(MS23, HIGH);
  digitalWrite(stp3, HIGH); //Trigger one step
  delay(steptime3);
  digitalWrite(stp3, LOW); //Pull step pin low so it can be triggered again
  delay(steptime3);
  z--;
}
void SmallStepBackwardMode4(){
  digitalWrite(EN4, LOW);
  digitalWrite(dir4, HIGH); //Pull direction pin low to move "forward"
  //digitalWrite(MS14, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  //digitalWrite(MS24, HIGH);
  digitalWrite(stp4, HIGH); //Trigger one step
  delay(steptime2);
  digitalWrite(stp4, LOW); //Pull step pin low so it can be triggered again
  delay(steptime2);
  pump_count++;
  Serial.println(steptime2);
}

void SmallStepBackwardMode5(){
  digitalWrite(EN5, LOW);
  digitalWrite(dir5, HIGH); //Pull direction pin low to move "forward"
  //digitalWrite(MS14, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  //digitalWrite(MS24, HIGH);
  digitalWrite(stp5, HIGH); //Trigger one step
  delay(steptime2);
  digitalWrite(stp5, LOW); //Pull step pin low so it can be triggered again
  delay(steptime2);
}

//shuttle mode will use the full speed between the initial site and work docking site. 
void SSF_shuttle1()
{
  digitalWrite(EN1, LOW);
  digitalWrite(dir1, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS11, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  digitalWrite(MS21, HIGH);
  digitalWrite(stp1, HIGH); //Trigger one step
  delay(1);
  digitalWrite(stp1, LOW); //Pull step pin low so it can be triggered again
  delay(1);
  x++;  
}
void SSF_shuttle2()
{
  digitalWrite(EN2, LOW);
  digitalWrite(dir2, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS12, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  digitalWrite(MS22, HIGH);
  digitalWrite(stp2, HIGH); //Trigger one step
  delay(1);
  digitalWrite(stp2, LOW); //Pull step pin low so it can be triggered again
  delay(1);
  y++;
}
void SSF_shuttle3()
{
  digitalWrite(EN3, LOW);
  digitalWrite(dir3, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS13, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  digitalWrite(MS23, HIGH);
  digitalWrite(stp3, HIGH); //Trigger one step
  delay(1);
  digitalWrite(stp3, LOW); //Pull step pin low so it can be triggered again
  delay(1);
  z++;
}

void SSB_shuttle1()
{
  digitalWrite(EN1, LOW);
  digitalWrite(dir1, HIGH); //Pull direction pin low to move "forward"
  digitalWrite(MS11, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  digitalWrite(MS21, HIGH);
  digitalWrite(stp1, HIGH); //Trigger one step
  delay(1);
  digitalWrite(stp1, LOW); //Pull step pin low so it can be triggered again
  delay(1);
  x--;
}
void SSB_shuttle2()
{
  digitalWrite(EN2, LOW);
  digitalWrite(dir2, HIGH); //Pull direction pin low to move "forward"
  digitalWrite(MS12, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  digitalWrite(MS22, HIGH);
  digitalWrite(stp2, HIGH); //Trigger one step
  delay(1);
  digitalWrite(stp2, LOW); //Pull step pin low so it can be triggered again
  delay(1);
  y--;
}
void SSB_shuttle3()
{
  digitalWrite(EN3, LOW);
  digitalWrite(dir3, HIGH); //Pull direction pin low to move "forward"
  digitalWrite(MS13, HIGH); //Pull MS11, and MS21 high to set logic to 1/8th microstep resolution
  digitalWrite(MS23, HIGH);
  digitalWrite(stp3, HIGH); //Trigger one step
  delay(1);
  digitalWrite(stp3, LOW); //Pull step pin low so it can be triggered again
  delay(1);
  z--;
}
void Pump_move_F(){
  digitalWrite(EN4, LOW);
  digitalWrite(dir4, LOW); //Pull direction pin low to move "forward", syringe volume will increase
  digitalWrite(stp4, HIGH); //Trigger one step
  delay(1); //2 mS period of the square wave 
  digitalWrite(stp4, LOW); //Pull step pin low so it can be triggered again
  delay(1);
  pump_count--;
  }
  
void Pump_move_B(){  
  digitalWrite(EN4, LOW);
  digitalWrite(dir4, HIGH); //Pull direction pin high to move "backward", syringe volume will increase
  digitalWrite(stp4, HIGH); //Trigger one step
  delay(1);
  digitalWrite(stp4, LOW); //Pull step pin low so it can be triggered again
  delay(1);
  pump_count++; 
  }
  
void Pump_move(){
  if(Pump_value > 0){
    for(int i = 0; i < abs(Pump_value); i++){
      Pump_move_F();}
    }
  if(Pump_value < 0){
    for(int i = 0; i < abs(Pump_value); i++){
      Pump_move_B();}
    }
  pump_state_count++;
  if(pump_state_count == 2){
    pump_state_count = 0;}
  }

void move_step(int eks,int wi,int zi,int count){
  if(eks < 0 && count == 1){              //return to the work docking point x-axis
    for(int i = 0; i < abs(eks); i++){
    SSF_shuttle1();
    }  
  }
  if(eks > 0 && count == 1){
    for(int i = 0; i < abs(eks); i++){
    SSB_shuttle1();
    }
  }
  if(wi < 0 && count == 1){              //return to the work docking point y-axis
    for(int i = 0; i < abs(wi); i++){
    SSF_shuttle2();
    }  
  }
  if(wi > 0 && count == 1){
    for(int i = 0; i < abs(wi); i++){
    SSB_shuttle2();
    }
  }
  if(zi < 0 && count == 1){              //return to the work docking point z-axis
    for(int i = 0; i < abs(zi); i++){
    SSF_shuttle3();
    }  
  }
  if(zi > 0 && count == 1){
    for(int i = 0; i < abs(zi); i++){
    SSB_shuttle3();
    }
  }
    
//if the shuttle switch is clicked twice, then the S_count will be equal to 2, and the the following will bring the tip to the last point.
  if(eks < 0 && count == 3){          //move the tip to the working x axis
    for(int i = 0; i < abs(eks); i++){
    SSB_shuttle1();
    }  
  }
  if(eks > 0 && count == 3){
    for(int i = 0; i < abs(eks); i++){
    SSF_shuttle1();
    }
  }
  if(wi < 0 && count == 3){          //move the tip to the default y axis
    for(int i = 0; i < abs(wi); i++){
    SSB_shuttle2();
    }  
  }
  if(wi > 0 && count == 3){
    for(int i = 0; i < abs(wi); i++){
    SSF_shuttle2();
    }
  }
  if(zi < 0 && count == 3){            
    for(int i = 0; i < abs(zi); i++){
    SSB_shuttle3();
    }  
  }
  if(zi > 0  && count == 3){
    for(int i = 0; i < abs(zi); i++){
    SSF_shuttle3();
    }
  }
}

void shuttle(){
  if(S_count == 1){  //cell was collected 
     a = x - arrayofPositions[pos_mem_count-1].xVal;  //if pos_mem_count ==2, then only one coordinate, i.e. arrayofPositions[1] was stored.
     b = y - arrayofPositions[pos_mem_count-1].yVal;
     c = z - arrayofPositions[pos_mem_count-1].zVal;
    move_step(a,b,c,S_count); //if the tip is moved away to the work docking site, this will bring the tip back to the work docking zone.
  }
  
  if(S_count == 3){      //if the tip is near or at the initial site({0,0,0})
    move_step(x,y,z,1); //"1" means the driver will drive the motor in reverse mode, while 3 means the motor work in forward mode. 
  }                     //this will bring the tip back to {0,0,0},if the tip deviate from the {0,0,0}
  
  for(int i = 0; i < pos_mem_count-1; i++){     //initial pos_mem_count == 1, if Coordinate_clk click once, the first coordinate will be stored and pos_mem_count == 2, this part is used to generate the relative position information.
    relativePostions[i].xVal = arrayofPositions[i+1].xVal - arrayofPositions[i].xVal;
    relativePostions[i].yVal = arrayofPositions[i+1].yVal - arrayofPositions[i].yVal;
    relativePostions[i].zVal = arrayofPositions[i+1].zVal - arrayofPositions[i].zVal;
    }
    
  if(S_count == 1){
    for(int i = pos_mem_count-1 ;i > 0; i--){
    move_step(relativePostions[i-1].xVal, relativePostions[i-1].yVal, relativePostions[i-1].zVal, S_count);
    }
  }//this will shuffle the tip back and forth between the work docking site and initial site.
  if(S_count == 3){
    for(int i = 0; i < pos_mem_count; i++){
    move_step(relativePostions[i].xVal, relativePostions[i].yVal, relativePostions[i].zVal, S_count);  
    }
  }
  S_count++;
  if(S_count==4){
    S_count=0;
    }
}

void resetEDPins()
{
  digitalWrite(stp1, LOW);
  digitalWrite(dir1, LOW);
  digitalWrite(MS11, LOW);
  digitalWrite(MS21, LOW);
  digitalWrite(EN1, HIGH);
  digitalWrite(stp2, LOW);
  digitalWrite(dir2, LOW);
  digitalWrite(MS12, LOW);
  digitalWrite(MS22, LOW);
  digitalWrite(EN2, HIGH);
  digitalWrite(stp3, LOW);
  digitalWrite(dir3, LOW);
  digitalWrite(MS13, LOW);
  digitalWrite(MS23, LOW);
  digitalWrite(EN3, HIGH);
  digitalWrite(stp4, LOW);
  digitalWrite(dir4, LOW);
  digitalWrite(MS14, LOW);
  digitalWrite(MS24, LOW);
  digitalWrite(EN4, HIGH);
  digitalWrite(stp5, LOW);
  digitalWrite(dir5, LOW);
  digitalWrite(MS15, LOW);
  digitalWrite(MS25, LOW);
  digitalWrite(EN5, HIGH);  
}
void print_coordinate(){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(1, 1);
  display.print("X:");
  display.print(x);
  display.setCursor(110, 1);
  display.print(CLICK_COUNT);
  display.setCursor(1, 21);
  display.print("Y:");
  display.print(y);
  display.setCursor(80, 21);
  display.print(Volume, 1);
  display.setCursor(1, 41);
  display.print("Z:");
  display.print(z);
  display.setCursor(110, 41);
  display.print(S_count);
  display.display();
  }

