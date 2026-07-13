#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Define aquí los pines que vas a usar físicamente
// Otros pines seguros: 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33
const int SDA_PIN = 17;
const int SCL_PIN = 16;

void setup(void) {
  Serial.begin(115200);

  // Inicialización manual del bus I2C con los pines definidos
  Wire.begin(SDA_PIN, SCL_PIN);

  // Inicializar sensor
  if (!mpu.begin()) {
    Serial.println("No se pudo encontrar un sensor MPU6050 válido, revisa las conexiones!");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 inicializado correctamente!");
  Serial.print("Pines I2C configurados -> SDA: ");
  Serial.print(SDA_PIN);
  Serial.print(" | SCL: ");
  Serial.println(SCL_PIN);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Imprimir datos en Monitor Serie
  Serial.print("Acc: X:"); Serial.print(a.acceleration.x); 
  Serial.print(" Y:"); Serial.print(a.acceleration.y); 
  Serial.print(" Z:"); Serial.print(a.acceleration.z); Serial.println(" m/s^2");

  Serial.print("Giro: X:"); Serial.print(g.gyro.x); 
  Serial.print(" Y:"); Serial.print(g.gyro.y); 
  Serial.print(" Z:"); Serial.print(g.gyro.z); Serial.println(" rad/s");

  delay(2000);
}