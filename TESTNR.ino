//****************************************//
//* ESP32 MQTT EEEBot Template           *//
//* Modified from Rui Santos'            *//
//* https://randomnerdtutorials.com      *//
//*                                      *//
//* ESP32 Code                           *//
//*                                      *//
//* UoN 2022 - ND                        *//
//****************************************//

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AM2320.h>
#include <Adafruit_SSD1306.h>


int leftEncoderCount;
int rightEncoderCount;

String motorSpeed;
String ServoAngleNodeRED;

int16_t leftMotorSpeed = 255;   //initiaising left motor speed
int16_t rightMotorSpeed = 255;  //initiaising right motor speed
int16_t servoAngle = 41;        //initiaising servo angle

float circumferenceOfWheels = 31.42;  // Measured circumference of wheels
float pulsesPerRevolution = 24;       // Number of pulses per revolution for rotary encoders

String distanceNR;

String LeftEncoderString;
String RightEncoderString;

//HCSR04
#define echoPin 5//
#define trigPin 18///
long duration;
int distance;

//MPU6050
MPU6050 mpu6050(Wire);
int mpux = 0;
int mpuy = 0;
int mpuz = 0;

//NEOPIXEL
#define LED_PIN 17 // Pin number for NeoPixel strip
#define LED_COUNT 15 // Number of LEDs in strip
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

//Infrared Sensor
int Sensor1 = 34;
int Sensor2 = 35;
int Sensor3 = 32;
int Sensor4 = 33;


int val1 = 0;
int val2 = 0;
int val3 = 0;
int val4 = 0;

//AM2320
Adafruit_AM2320 AM2320 = Adafruit_AM2320();


//OLED  
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDR 0x3C // OLED display address
#define OLED_SDA 21 // OLED display SDA pin
#define OLED_SCL 22 // OLED display SCL pin
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT);

//

// Replace the next variables with your SSID/Password combination
const char* ssid = "B6-RaspberryPiFi";
const char* password = "b6channer";

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "192.168.2.1";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
int value = 0;
char msg[50];

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
 
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT
  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
       for(int j=0; j<256; j++) { // cycle through all possible colors
    for(int i=0; i<LED_COUNT; i++) {
      strip.setPixelColor(i, strip.Color(255-j,j,0)); // color wheel transition
      strip.show();
    }
    delay(50);
  }
}
    else if(messageTemp == "off"){
    Serial.println("off");
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.show();  
    }
  }
   if (String(topic) == "esp32/motorSpeed") {
    motorSpeed = messageTemp;
  }else if (String(topic) == "esp32/servoAngle") {
    ServoAngleNodeRED = messageTemp;
  }else if (String(topic) == "esp32/distance") {
    distanceNR = messageTemp;
  }
 
}
void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);   // Sets the echoPin as an Input
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  strip.begin();
  strip.show();
  AM2320.begin();
   display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Initialize the OLED display
  display.clearDisplay();  // Clear the display buffer
}

void setup_wifi(){
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}




void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");

      // Add your subscribe topics here
      // --
      client.subscribe("esp32/output");
      client.subscribe("esp32/distance");
      client.subscribe("esp32/motorSpeed");
      client.subscribe("esp32/servoAngle");

      // --

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {
  
     Wire.requestFrom(0x04, 2);  // Request 2 bytes of data from the Arduino Nano
  if (Wire.available() >= 2) {
    leftEncoderCount = Wire.read();
    rightEncoderCount = Wire.read();
  }
  Serial.println(leftEncoderCount);
  Serial.println(rightEncoderCount);

  
    if (!client.connected()) {
    reconnect();
  }
  client.loop();

    LeftEncoderString = String(leftEncoderCount);
    RightEncoderString = String(rightEncoderCount);
    client.publish("esp32/leftEncoder", LeftEncoderString.c_str());
    client.publish("esp32/rightEncoder", RightEncoderString.c_str());


  long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;S


    //HC-SR04
     digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);

    // Calculate the distance
    distance = duration * 0.034 / 2;
    Serial.println(distance);
    char distanceString[8];

    dtostrf(distance, 1, 2, distanceString);
    client.publish("esp32/distance", distanceString);

    //MPU6050
    //Angle X
    mpu6050.update();
    mpux = mpu6050.getAngleX();
    Serial.println(mpux);
    char mpuxString[8];
    dtostrf(mpux, 1, 2, mpuxString);
    client.publish("esp32/Angle_X", mpuxString);
    //Angle Y
     mpuy = mpu6050.getAngleY();
    Serial.println(mpuy);
    char mpuyString[8];
    dtostrf(mpuy, 1, 2, mpuyString);
    client.publish("esp32/Angle_Y", mpuyString);

    // Angle Y
     mpuz = mpu6050.getAngleZ();
    Serial.println(mpuz);
    char mpuzString[8];
    dtostrf(mpuz, 1, 2, mpuzString);
    client.publish("esp32/Angle_Z", mpuzString);
 
    //IR S1
    val1 = analogRead(Sensor1);
     Serial.println(val1);
    char S1String[8];
    dtostrf(val1, 1, 2, S1String);
    client.publish("esp32/Sensor1", S1String);

    //IR S2
    val2 = analogRead(Sensor2);
    Serial.println(val2);
    char S2String[8];
    dtostrf(val2, 1, 2, S2String);
    client.publish("esp32/Sensor2", S2String);

    //IR S3
    val3 = analogRead(Sensor3);
    Serial.println(val3);
    char S3String[8];
    dtostrf(val3, 1, 2, S3String);
    client.publish("esp32/Sensor3", S3String);

    //IR S4
    val4 = analogRead(Sensor4);
    Serial.println(val4);
    char S4String[8];
    dtostrf(val4, 1, 2, S4String);
    client.publish("esp32/Sensor4", S4String);

    //AM2320 Temp
    float tempC = AM2320.readTemperature();
    Serial.println(tempC);
    char TempString[8];
    dtostrf(tempC, 1, 2, TempString);
    client.publish("esp32/Temperature", TempString);

     display.setTextSize(1);  
    display.setTextColor(WHITE);  
    display.setCursor(0, 0);  
    display.print("Sensor 3");
    display.setCursor(80, 0);  
    display.print("Sensor 4");
   
    // Print sensor values
    display.setTextSize(2);  
    display.setCursor(0, 40);  
    display.print(val3);  
    display.setCursor(80, 40);  
    display.print(val4);  
   
   
    display.display();
    delay(100);
    display.clearDisplay();

  leftMotorSpeed = (int16_t)motorSpeed.toInt();
  rightMotorSpeed = (int16_t)motorSpeed.toInt();
  servoAngle = (int16_t)ServoAngleNodeRED.toInt();

  //Communicate with arduino
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);  // Limit left motor speed between -255 and 255
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);  // Limit right motor speed between -255 and 255
  servoAngle = constrain(servoAngle, 0, 180);  // Limit servo angle between 0 and 180

  Wire.beginTransmission(0x04);        // Prepare and send control values to the I2C slave device with address 4
  Wire.write(leftMotorSpeed >> 8);     // Send the first byte of the left motor speed control value
  Wire.write(leftMotorSpeed & 0xff);   // Send the second byte of the left motor speed control value
  Wire.write(rightMotorSpeed >> 8);    // Send the first byte of the right motor speed control value
  Wire.write(rightMotorSpeed & 0xff);  // Send the second byte of the right motor speed control value
  Wire.write(servoAngle >> 8);         // Send the first byte of the servo angle control value
  Wire.write(servoAngle & 0xff);       // Send the second byte of the servo angle control value
  Wire.endTransmission();              // End the transmission and send the data to the I2C slave device

   
  }
}