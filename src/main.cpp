/*
 * See documentation at https://nRF24.github.io/RF24
 * See License information at root directory of this library
 * Author: Brendan Doherty (2bndy5)
 */

/**
 * A simple example of sending data from 1 nRF24L01 transceiver to another.
 *
 * This example was written to be used on 2 devices acting as "nodes".
 * Use the Serial Monitor to change each node's behavior.
 */
#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define NRF_Transmit_first
//#define NRF_Receive_first

#define ID_transmiter 0xAA
#define ID_my_transmiter 0xEE
// instantiate an object for the nRF24L01 transceiver
RF24 radio(19, 18);  // using pin 7 for the CE pin, and pin 8 for the CSN pin

// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
#ifdef NRF_Transmit_first
  bool role = true;  // true = TX role, false = RX role
#endif
#ifdef NRF_Receive_first
  bool role = false;  // true = TX role, false = RX role
#endif

/*Data Tx[0] gởi ACK cho receiver
       Tx[1] gia tri adc 1
       Tx[2] gia tri adc 2
       Tx[3] gia tri adc 3
       Tx[4] gia tri adc 4
       */ 
int16_t payload_TX[5];
/*Data Tx[0] gia tri adc 1
       Tx[1] gia tri adc 2
       Tx[2] gia tri adc 3
       Tx[3] gia tri adc 4
       Tx[5] gởi ACK cho receiver
       */ 
int16_t payload_RX[5];
void setup() {

  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
  payload_TX[0]=ID_my_transmiter;//ACK of transmiter
  //payload_RX[0]=0xAA;//ID of receiver
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }

  // print example's introductory prompt
  Serial.println(F("RF24 getting started"));
#ifdef NRF_Transmit_first
  radioNumber  = 1;
#endif
#ifdef NRF_Receive_first
   radioNumber  = 0;
#endif
  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  radio.setPayloadSize(sizeof(payload_TX));  // float datatype occupies 4 bytes

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);  // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1

  // additional setup specific to the node's role
  if (role) {
    radio.stopListening();  // put radio in TX mode
  } else {
    radio.startListening();  // put radio in RX mode
  }
}  // setup

void loop() {
  payload_TX[1]+=1;
  payload_TX[2]=2;
  payload_TX[3]=3;
  payload_TX[4]=4;
  if (role) {
    // This device is a TX node

    unsigned long start_timer = micros();                // start the timer
    bool report = radio.write(&payload_TX, sizeof(payload_TX));  // transmit & save the report
    unsigned long end_timer = micros();                  // end the timer

    if (report) {
//    Serial.print(F("payload_TX :   "));  // payload was delivered
      radio.startListening();
      role = false;//switch to receiver
    } else {
      //Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
    }

  } else {if(role==false){
      // This device is a RX node

      uint8_t pipe;
      if (radio.available(&pipe)) {              // is there a payload? get the pipe number that recieved it
        uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
        radio.read(&payload_RX, bytes);             // fetch payload from FIFO
        if (payload_RX[0] = ID_transmiter){
          Serial.print(F("payload_RX   :     "));
          Serial.print(payload_RX[1]);  // print the payload's value
          Serial.print("    ");
          Serial.print(payload_RX[2]);
          Serial.print("    ");
          Serial.print(payload_RX[3]);
          Serial.print("    ");
          Serial.println(payload_RX[4]);
        
          radio.stopListening();
          role = true;//switch to transmiter
        }
      }
      else{
        //Serial.println("receiver mode - no signal ");
      }
    }
  } 
  //delay(50);
}  // loop
