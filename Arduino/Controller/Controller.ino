#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <printf.h>
#define MAX_SIZE 6
#define CE_PIN 7
#define CSN_PIN 8
#define FC_NODE 1
#define CONTROLLER_NODE 0
#define TYPE_ANGLE 0 
#define TYPE_CONTROL 1
#define TYPE_PID 2
#define PIN_FRONT 3
#define PIN_BACK 5
#define PIN_LEFT 6
#define PIN_RIGHT 9

RF24 radio(CE_PIN,CSN_PIN);
RF24Network network(radio);

struct payload_t
{
  double dataVector[MAX_SIZE];
};
double time = 0;
int lastNumber = 0;
int missed = 0;
int joystick;
void setup(void)
{
  pinMode(2,OUTPUT);
  Serial.begin(115200);
  Serial.println("RF24Network/examples/helloworld_rx/");
  printf_begin();

  SPI.begin();
  radio.begin();
  network.begin(/*channel*/ 90, /*node address*/ CONTROLLER_NODE);
}

void loop(void)
{

  // Pump the network regularly
  network.update();

  // Is there anything ready for us?
  if( network.available() )
  {
    readData(); 
  }

  if(Serial.available())
  {
    readSerial();
  }

  int tmp = analogRead(0);
  if(abs(joystick - tmp) > 3 && millis() - time > 50 )
  {
    joystick = tmp;
    double value[1] = {map(joystick,0,1023,80,245)};
    sendData(value,TYPE_CONTROL);
    time = millis();
    
  }  

}
void sendData(double data[5],uint8_t type)
{
  network.update();
  payload_t payload;

  switch(type)
  {
  case TYPE_CONTROL:
    {
      payload.dataVector[0] = data[0]; 
      RF24NetworkHeader header(/*to node*/ FC_NODE,TYPE_CONTROL);
      while(!network.write(header,&payload,sizeof(payload)));
      break;
    }
  case TYPE_PID:
    {
      payload.dataVector[0] = data[0]; 
      payload.dataVector[1] = data[1]; 
      payload.dataVector[2] = data[2]; 
      RF24NetworkHeader header(/*to node*/ FC_NODE,TYPE_PID);
      while(!network.write(header,&payload,sizeof(payload)));
    }
    break;
  }

}
void readData()
{
  RF24NetworkHeader header;
  payload_t payload;
  network.read(header,&payload,sizeof(payload));

  switch (header.type)
  {
  case 0:          
    Serial.print(payload.dataVector[0]);
    Serial.print(" \t");
    Serial.print(payload.dataVector[1]);
    Serial.print(" \t");
    Serial.print(payload.dataVector[2]);
    Serial.print(" \t");
    Serial.print(payload.dataVector[3]);
    Serial.println("");
    break;
  case 1:
    Serial.println(payload.dataVector[0]);
    break;
  }
}
void readSerial()
{
  switch(Serial.read())
  {
  case 'a':
    double value[3]  = {getValue(),getValue(),getValue()};
    sendData(value,TYPE_PID);
    break;
  }
}
double getValue()
{
  double value;
  int  nDigits;
  double integer = Serial.parseInt();
  double decimal = Serial.parseInt();
  if(decimal == 0)
    nDigits = 1;
  else
    nDigits = floor(log10(abs(decimal))) + 1; 
  value = integer + decimal/(pow(10,nDigits));
  return value;
}

// vim:ai:cin:sts=2 sw=2 ft=cpp



