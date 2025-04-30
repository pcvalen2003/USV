//////// Arduino MEGA ////////

// La librería de RF24 usa las funcionalidades de Arduino.h
#include <Arduino.h>
#include <RF24.h>

#define CE 49
#define CSN 48

typedef struct{
  uint16_t x;
  uint16_t y;
} mensaje_t;

const uint64_t direccion = 0xE8E8F0F0E1LL;

mensaje_t mensaje;

RF24 radio(CE, CSN);

// Prototipo para inicializar Arduino internamente (para el RF24)
extern "C" void init();


int main() {
  init(); // Inicializa las funcionalidades de Arduino.h necesarias para el RF24

  Serial.begin(9600);
  if (radio.begin() == 0) {
    Serial.println("error de radio");
  } else Serial.println("Radio iniciada");

  
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, direccion);
  radio.startListening();


  while(1) {
    if (radio.available()) {
      radio.read(&mensaje, sizeof(mensaje));
      Serial.print("\nCoord X: ");
      Serial.print((int)mensaje.x);
      Serial.print("\tCoord Y: ");
      Serial.print((int)mensaje.y);  
    }
  }

}

