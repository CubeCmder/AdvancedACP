#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(9, 10); // CE, CSN         
const byte address[6] = "00001";     //Byte of array representing the address. This is the address where we will send the data. This should be same on the receiving side.
int button_pin = 2;
boolean button_state = 0;

int altitude = 1000;

void setup() {
pinMode(button_pin, INPUT);
radio.begin();                  //Starting the Wireless communication
radio.openWritingPipe(address); //Setting the address where we will send the data
radio.setPALevel(RF24_PA_MAX);  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
radio.stopListening();          //This sets the module as transmitter
Serial.begin(9600);
}
void loop()
{
  // button_state = digitalRead(button_pin);
  // Serial.println(button_state);
  // if(button_state == HIGH)
  //   {
  //   const char text[] = "Your Button State is HIGH";
  //   radio.write(&text, sizeof(text));                  //Sending the message to receiver
  //   radio.write(&altitude, sizeof(altitude));
  //   radio.write(&button_state, sizeof(button_state));
  //   }
    
  radio.setRetries(0,0); // by default nrf tries 15 times. Change to no retries to measure strength

  char buffer[32];
  int counter = 0;

  for(int i=0; i<100; i++)
  {
    int status = radio.write(&buffer, sizeof(buffer)); // send 32 bytes of data. It does not matter what it is
    if(status)
        counter++;

    delay(1); // try again in 1 millisecond
  }
  //Serial.println(counter);
  if (counter > 80) //signal strength is great
  {
    Serial.print("Signal is good: ");
    Serial.print(counter);
    Serial.println("% packets received");
  }

  // etc

  //radio.setRetries(5,15);

  delay(50);
}
