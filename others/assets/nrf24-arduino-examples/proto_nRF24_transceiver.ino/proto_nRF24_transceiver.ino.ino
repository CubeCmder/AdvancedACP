#include<SPI.h>                   // spi library for connecting nrf
#include<RF24.h>                  // nrf library


RF24 radio(9, 10) ;  // ce, csn pins    

void setup(void) {

  while (!Serial) ;

  Serial.begin(115200) ;     // start serial monitor baud rate

  Serial.println("Starting.. Setting Up.. Radio on..") ; // debug message

  radio.begin();        // start radio at ce csn pin 9 and 10
  radio.setChannel(0x36) ;            // set chanel at 76
  const uint64_t pipe = 0xE0E0E0E0E0LL ;    // pipe address same as sender i.e. raspberry pi
  radio.openWritingPipe(pipe) ;        // start reading pipe 

  radio.enableDynamicPayloads() ;
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN) ;   // set power level

  radio.powerUp() ;          
  radio.stopListening();          //This sets the module as transmitter
  

}


void loop(void) {

  Serial.println("Starting.. Setting Up.. Radio on..") ;
  const char text[] = "Your Button State is HIGH";
  radio.write(&text, sizeof(text));                  //Sending the message to receiver

  delay(50);

}