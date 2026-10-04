#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

#define ce_pin 9
#define csn_pin 10
#define thro_pot_pin A1
#define st_pot_pin A0
#define switch_pin 8
#define interval 10

RF24 radio(ce_pin, csn_pin);

const byte address[5] = {'R','x','A','A','A'};

struct noask_packet {
  int32_t steering;
  int32_t throttle;
};
noask_packet noaskp1;

struct ask_packet {
  bool forward;
};
ask_packet askp1;

void setup() {
  Serial.begin(115200);
  pinMode(switch_pin, INPUT_PULLUP);
  askp1.forward = true;
  /*if (!radio.begin()) {                         // check if it can communicate with nrf
    Serial.println("RF24 not found!");
    while(1); }*/
  radio.begin();
  radio.openWritingPipe(address);
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);
  radio.enableDynamicAck();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_2MBPS);
  radio.stopListening();
}

void loop() {
  noaskp1.steering = analogRead(st_pot_pin);
  noaskp1.throttle = analogRead(thro_pot_pin);
  askp1.forward = digitalRead(switch_pin) == HIGH;
  radio.write(&noaskp1, sizeof(noaskp1), false);
  bool sentaskpacket = false;
  while(!sentaskpacket) {
    sentaskpacket = radio.write(&askp1, sizeof(askp1), true);
    if (!sentaskpacket) {
      Serial.println("ask packet not sent, retrying...");
      delay(4);
    }
  }
  Serial.print("steer:");
  Serial.println(noaskp1.steering);
  Serial.print("           throt:");
  Serial.println(noaskp1.throttle);
  Serial.print("                        forward:");
  Serial.println(askp1.forward);
  delay(interval);
}
