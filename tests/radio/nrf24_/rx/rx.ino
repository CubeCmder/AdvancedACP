#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";
boolean button_state = 0;
int altitude = 0;
char buffer[32];
int led_pin = 3;
void setup() {
  pinMode(6, OUTPUT);
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MAX);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.startListening();              //This sets the module as receiver
}
void loop()
{
  if (radio.available())              //Looking for the data.
  {
    char text[32] = "";                 //Saving the incoming data
    radio.read(&text, sizeof(text));    //Reading the data
    radio.read(&altitude, sizeof(altitude));    //Reading the data (altitude)
    radio.read(&button_state, sizeof(button_state));    //Reading the data
    if (button_state == HIGH)
    {
      Serial.println("Altitude is "+ String(altitude));
    }
  }
  delay(1);
}
