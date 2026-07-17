#include <WiFi.h>

void setup() {
  // Inicializar el puerto serie a 115200 baudios
  Serial.begin(115200);
  delay(1000); 

  // Activar el Wi-Fi en modo Estación (STA) para inicializar la interfaz de red y la MAC
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.println("");
  Serial.println("==================================================");
  Serial.println("    ESCUDE RÍA MECATRÓNICA - EQUIPO DE TIERRA     ");
  Serial.println("==================================================");
  Serial.print("  DIRECCIÓN MAC DE TU ESP32 (38 PINES): ");
  Serial.println(WiFi.macAddress());
  Serial.println("==================================================");
}

void loop() {
  // Dejamos el bucle vacío con un delay para que no sature
  delay(1000);
}