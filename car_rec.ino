#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Servo.h>

#define bat_pin PA1

#define LED_pin PC13

#define ce_pin PB0
#define csn_pin PB1

#define servo_pin PA2

#define ML1_pin PB6
#define ML2_pin PB7
#define MR1_pin PB8
#define MR2_pin PB9

#define b 11.5 // net width of car
#define l 12 // wheel base of car

#define n 8
int32_t th_templis[n];
int32_t st_templis[n];

int32_t throtAvg = 0;
int32_t steerAvg = 0;

unsigned long lastrxtime;

RF24 radio(ce_pin, csn_pin);
Servo s1;

const byte address[5] = {'R','x','A','A','A'};

struct noask_packet {
  int32_t steering;
  int32_t throt9tle;
};
noask_packet noaskp1;

struct ask_packet {
  bool forward;
};
ask_packet askp1;

void nzeroarray (int32_t *lis){
  for(int i = 0; i < n; i++) {
    lis[i] = 0;
  }
}

void adjust_extreems (int32_t &T, int32_t &S){
  if (S>230 && S<270){
    S = 256;
  }
  if (S>470){
   S = 511;
  }
  if (S<35){
    S = 0;
  }
  if (T<80){
    T = 0;
  }
  if (T>476){
    T = 511;
  }
}

/*void bat_volt (){
  if (analogRead(bat_pin))
}*/

int32_t MovAvg (int32_t inp, int32_t *templis){
  int32_t sum = 0;
  for(int i = n-1; i > 0; i--) templis[i] = templis[i-1];
  templis[0] = inp;
  for(int i = 0; i < n; i++) sum += templis[i];
  return sum/n;
}

void set_motors (int32_t T, int32_t S, bool forward){
  S = map(constrain(abs(S-255),0,255),0,255,0,(3.14/4));
  int To, Ti;
  int k = ((b/(10*l))*S*(3*S + 4)); // S should be in radians
  if (T >= (511-k*T)){
    To = T;
    Ti = T*(1 - 2*k);
  }
  else if (T <= k*T) {
    To = T*(1 + 2*k);
    Ti = T;
  }
  else {
    To = T*(1 + k);
    Ti = T*(1 - k);
  }
  int Mopwm = map(To,0,511,0,225);
  int Mipwm = map(Ti,0,511-(2*k),0,225);
  if (S-255 < 0){
    if(forward) {
    analogWrite(ML1_pin, Mipwm);
    analogWrite(ML2_pin, 0);
    analogWrite(MR1_pin, Mopwm);
    analogWrite(MR2_pin, 0);
  } else {
    analogWrite(ML1_pin, 0);
    analogWrite(ML2_pin, Mipwm);
    analogWrite(MR1_pin, 0);
    analogWrite(MR2_pin, Mopwm);
    }
  } else {
    if(forward) {
    analogWrite(ML1_pin, Mopwm);
    analogWrite(ML2_pin, 0);
    analogWrite(MR1_pin, Mipwm);
    analogWrite(MR2_pin, 0);
  } else {
    analogWrite(ML1_pin, 0);
    analogWrite(ML2_pin, Mopwm);
    analogWrite(MR1_pin, 0);
    analogWrite(MR2_pin, Mipwm);
    }
  }
}

void setup() {
  nzeroarray(th_templis);
  nzeroarray(st_templis);
  pinMode(LED_pin, OUTPUT);
  Serial.begin(115200);
  s1.attach(servo_pin);
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);
  radio.enableDynamicAck();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_2MBPS);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    uint8_t len = radio.getDynamicPayloadSize();
    if(len == sizeof(noask_packet)) {
      radio.read(&noaskp1, sizeof(noaskp1));
      lastrxtime = millis();
      digitalWrite(LED_pin, HIGH); // signal ok no blink
      throtAvg = constrain(MovAvg(noaskp1.throttle, th_templis),0,1023)>>1;
      steerAvg = constrain(MovAvg(noaskp1.steering, st_templis),0,1023)>>1;
    }
    else if(len == sizeof(ask_packet)) {
      radio.read(&askp1, sizeof(askp1));
      lastrxtime = millis();
      digitalWrite(LED_pin, HIGH); // signal ok no led
    }
  }
  adjust_extreems(throtAvg, steerAvg);

  set_motors(throtAvg, steerAvg, askp1.forward);

  s1.write((map(steerAvg,0,511,45,135)));
  Serial.println("steerAvg");
/*// for debugining.
 Serial.print("stree:");
  Serial.println(steerAvg);
  Serial.print("            throt:");
  Serial.println(throtAvg);
  Serial.print("                         forward:");
  Serial.println(askp1.forward);
  Serial.print("                                     voltage per cell:");
  Serial.println(analogRead((bat_pin)/1023)*3.3);*/

  if (millis() - lastrxtime > 500){
    digitalWrite(LED_pin, LOW);
    set_motors(0, 0, true);
    //s1.write(90);
  }
}
