#include <RF24.h> 
#include <WiFi.h>


#define CE 2
#define CSN 15  
RF24 radio(CE, CSN);


char mensaje = 'A';
const uint64_t direccion = 0xE8E8F0F0E1LL;





void setup() {
Serial.begin(115200);

char nrf24_check = radio.begin();
  
 // Si el NRF no se inicializó, tiro error y stalleo
if (nrf24_check == 0) {
       Serial.println("Error radio");
    while (1) {
      // stall
    }
} else Serial.println("Radio iniciada");

  
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(direccion);

}

void loop() {
  
  radio.write(&mensaje,sizeof(mensaje));
  Serial.println(mensaje);
  delay(1000);

}
