/* Minimum_Source*/

// Dynamixel on Serial1 (USART1)
#define DXL_BUS_SERIAL1 1

// define servo ID
#define ID 1

Dynamixel Dxl(DXL_BUS_SERIAL1);

// set analog pin
int pin0 = 0;

int val = 0;
int pos = 500;
int posMin = 200;
int posMax = 1000;
int count = 0;

void setup() {
  Dxl.begin(3);  // 1Mhz Baudrate
  Dxl.jointMode(ID); // set servo1 to joint mode
  
  pinMode(pin0, INPUT); // set pin0 to input
  
  SerialUSB.begin();
}

void loop() {
  delay(10);
  val = analogRead(pin0);  // return value 0-1023
  val = val/4;
  
  // for debug - serial monitor
  SerialUSB.print("Potentiometer value: ");
  SerialUSB.println(val);
  SerialUSB.print("Position: ");
  SerialUSB.println(pos);
    
  delay(10);
  // sets position based on potentiometer position
  if(val > 1023){}
  else if(val < posMin)
    pos = posMin;
  else if(val > posMax)
    pos = posMax;
  else
    pos = val;
  
  delay(10);
  Dxl.goalPosition(ID, pos);  // move servo to pos
}
