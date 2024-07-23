
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL3e385lJ0O"
#define BLYNK_TEMPLATE_NAME "E AGRI"
#define BLYNK_AUTH_TOKEN "UV4EAwvk608rZweUJwQpvyevyS_w4H4u"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Servo.h>


char auth[] = "UV4EAwvk608rZweUJwQpvyevyS_w4H4u";
char ssid[] = "EAGRI";  // type your wifi name
char pass[] = "aaaaaaaa";  // type your wifi password

BlynkTimer timer; 
#define DHTPIN 0 //Connect Out pin to D3 in NODE MCU
#define DHTTYPE DHT11  
DHT dht(DHTPIN, DHTTYPE);
int M1F = D1; //GPIO5
int M1R = D2; //GPIO4
#define ONE_WIRE_BUS 14 // GPIO pin where the DS18B20 is connected
#define IR_SENSOR_PIN D6
#define SERVO_PIN D0
#define RELAY_PIN D4

Servo servo;  // Include the Servo library

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

bool isRelayOn = false;



void sendSensor()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
    Blynk.virtualWrite(V3, t);
    Blynk.virtualWrite(V4, h);
    Serial.print("Temperature : ");
    Serial.print(t);
    Serial.print("    Humidity : ");
    Serial.println(h);
  
}



void setup(){
  
  pinMode(M1F, OUTPUT);
  pinMode(M1R, OUTPUT);

  Serial.begin(9600);
  Blynk.begin(auth,ssid,pass);
  dht.begin();
  timer.setInterval(100L, sendSensor);
  sensors.begin();
  timer.setInterval(100L, readTemperature);
    servo.attach(SERVO_PIN);
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
    timer.setInterval(100L, checkIRSensor);
  
}

void loop(){
  Blynk.run();
  timer.run();
 
}

void readTemperature() {
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0); // Assuming only one DS18B20 sensor
  Blynk.virtualWrite(V5, temperature); // Use a Blynk widget (e.g., Value Display) to display the temperature on V5
}


void checkIRSensor() {
  int irStatus = digitalRead(IR_SENSOR_PIN);
  
  if (irStatus == HIGH) {
    if (!isRelayOn) {
      // IR sensor is high, perform the actions
      servo.write(180);  // Rotate servo to 90 degrees      
      digitalWrite(RELAY_PIN, HIGH);  // Turn on relay
      isRelayOn = true;
      
      timer.setTimeout(3000L, turnOffRelay);  // Turn off the relay and servo after 3 seconds
    }
  } else {
    // IR sensor is low
    isRelayOn = false;
    digitalWrite(RELAY_PIN, LOW);  // Ensure relay is off
    servo.write(0);  // Return servo to 0 degrees
  }
}

void turnOffRelay() {
  digitalWrite(RELAY_PIN, LOW);  // Turn off the relay
  servo.write(0);  // Return servo to 0 degrees
}
