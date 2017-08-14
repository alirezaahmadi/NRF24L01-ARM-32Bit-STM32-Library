
#include <SPI.h>
#include <NRF24.h>

NRF24 radio;

bool tx;
uint8 buf[21];
uint8 tx_buf[21]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
byte conn=0;
void setup()
{
  conn=1;
  delay(1000);
  SerialUSB.println("NRF24 Broadcast example");
  SerialUSB.println(tx);
  pinMode(18, INPUT);
  if(radio.begin(24, 25)){SerialUSB.println("Fail!!");conn=0;}

  // Pin 7 sets the mode (Sender or Receiver). Connect to GND on the sender
  //pinMode(0, INPUT_PULLUP);
  tx = 1;

  if (tx)
  {
    radio.setAddress(0xD2);
  }
  else
  {
    radio.listenToAddress(0xD2);
  }
  SerialUSB.print("TX mode: ");
  SerialUSB.println(tx); 
}

void loop()
{
  if (tx)
  {
    SerialUSB.println("Broadcasting.. ");
    radio.broadcast(tx_buf,21);
    //bool sent = radio.send(0xD2, "Hello there receiver 0xD2!");
    toggleLED();
    radio.flushTX();
    radio.flushRX();
  }
  else if (radio.available())
  {
    uint8 numBytes = radio.read(buf, sizeof(buf));
    if(numBytes == 66){
    SerialUSB.print("Received ");
    SerialUSB.print(numBytes);
    SerialUSB.print(" bytes: ");
    for(byte i=0;i<21;i++)
    SerialUSB.print(buf[i]);
    SerialUSB.println();
    for(byte i=0;i<21;i++)buf[i]=0;
    radio.flushTX();
    radio.flushRX();
    toggleLED();
    }
  }
  delay(1);
}
