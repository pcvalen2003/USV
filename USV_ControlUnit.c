#include <RF24.h> 
#define CE 49
#define CSN 48

const uint64_t direccion = 0xE8E8F0F0E1LL;
char mensaje;

RF24 radio(CE,CSN);

void setup() {
  Serial.begin(9600); 
  if(radio.begin() == 0){
    Serial.println("error de radio");
    while(1){}
  }else Serial.println("Radio iniciada");
  
    
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1,direccion);
  radio.startListening();           
}

void loop() {
  if(radio.available()){
    radio.read(&mensaje,sizeof(mensaje));
    Serial.println(mensaje);
    }
}
