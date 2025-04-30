//////// ESP32 ////////
#include "RF24.h"
#include <WiFi.h>


#define CE 2
#define CSN 15
#define ADC_x 39
#define ADC_y 34

RF24 radio(CE, CSN);

typedef struct {
  uint16_t x;
  uint16_t y;
} mensaje_t;

const uint8_t direccion[5] = {0xE1, 0xF0, 0xF0, 0xE8, 0xE8};

mensaje_t mensaje;

void setup() {
  Serial.begin(115200);
  char nrf24_check = radio.begin();

  /*Declaro los pindes de ADC*/
  pinMode(ADC_x, INPUT);
  pinMode(ADC_y, INPUT);

  // Si el NRF no se inicializó, tiro error y stalleo
  if (nrf24_check == 0) {
    Serial.println("Error radio");
    while (1) { /* stall */ }
  } else Serial.println("Radio iniciada");


  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(direccion);
  radio.setChannel(76);
  radio.setPALevel(RF24_PA_LOW);
}

  void loop() {
    mensaje.x = analogRead(ADC_x);
    mensaje.y = analogRead(ADC_y);

    radio.write(&mensaje, sizeof(mensaje));
    Serial.print("\nCoord X: ");
    Serial.print((int)mensaje.x);
    Serial.print("\tCoord Y: ");
    Serial.print((int)mensaje.y);
    delay(100);
  }
