/*********
 ESP8266 collect data from sensors and 
 publish value results on self Web server and 
 send to MQTT broker 
*********/

#include <Wire.h>
#include <ESP8266WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <PubSubClient.h>


Adafruit_BMP085 bmp; // I2C

// Replace with your network details
const char* ssid = ".....";
const char* password = ".....";

const char* mqtt_server = "....."; // Name MQTT server
const int mqtt_port = 1111; // Port MQTT server
const char* mqtt_user = "....."; // User name
const char* mqtt_pass = "....."; // Password

//mqtt setup
WiFiClient wclient;
PubSubClient client_mqtt(wclient, mqtt_server, mqtt_port);

float a, t, p, mrt, pc, pin;
long lastMsg = 0;

// Web Server on port 80
WiFiServer server(80);

void setupWifi(){
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
}

void connectSensor() {
  Wire.begin(4, 5);
  Wire.setClock(100000);
  //BMP085 connect
  Serial.println(F("BMP085 test"));
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1);
  }
}

void reconnectMqtt() {
  Serial.println("Connecting to MQTT server");
  if (client_mqtt.connect(MQTT::Connect("arduinoClient2")
          .set_auth(mqtt_user, mqtt_pass))) {
    Serial.println("Connected to MQTT server");
    // 
    client_mqtt.set_callback(callback);
    client_mqtt.subscribe("pushbutton"); 
    // send data
    getWeather();
    dataSend();
  } else {
    Serial.println("Could not connect to MQTT server");
  }
}

// receive data from mqtt server
void callback(const MQTT::Publish& pub){
  // serial print topic name
  Serial.print(pub.topic());
  Serial.print(" => ");
  // serial print value
  Serial.println(pub.payload_string()); 
  String payload = pub.payload_string();
  if(payload == "1") {
    // send data
    getWeather();
    dataSend();
  }
}

void dataSend() {
  //send temerature
  client_mqtt.publish("temperature",String(t));
  Serial.print("Send mqtt server topic \"Temprature\" value: ");
  Serial.println(t);
  //send pressure
  client_mqtt.publish("pressure",String(mrt));
  Serial.print("Send mqtt server topic \"Pressure\" value: ");
  Serial.println(mrt);
}

void getWeather() {
  t = bmp.readTemperature();
  p = bmp.readPressure();
  mrt = p * 0.0075;
  a = bmp.readAltitude();
  pc = bmp.readSealevelPressure();
  Serial.print("Temprature = ");
  Serial.println(t);
  Serial.print("Pressure = ");
  Serial.println(p);
  Serial.print("Pressure mrt = ");
  Serial.println(mrt);
  Serial.print("Altitude = ");
  Serial.println(a);
  Serial.print("SealevelPressure = ");
  Serial.println(pc);
  delay(100);
}

void serverTask() {
  // Listenning for new clients
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New client");
    // bolean to locate when the http request ends
    boolean blank_line = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == '\n' && blank_line) {
            getWeather();
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html");
            client.println("Connection: close");
            client.println();
            // your actual web page that displays temperature
            client.println("<!DOCTYPE HTML>");
            client.println("<html>");
            client.println("<head><META HTTP-EQUIV=\"refresh\" CONTENT=\"15\"></head>");
            client.println("<body><h1>ESP8266 Weather Web Server</h1>");
            client.println("<table border=\"2\" width=\"456\" cellpadding=\"10\"><tbody><tr><td>");
            client.println("<h3>Temperature = ");
            client.println(t);
            client.println("&deg;C</h3><h3>Pressure = ");
            client.println(p);
            client.println("Pa</h3><h3>Pressure mrt = ");
            client.println(mrt);
            client.println("m.r.t.</h3><h3>Altitude = ");
            client.println(a);
            client.println("meter</h3><h3>Pressure sea level = ");
            client.println(pc);
            client.println("Pa</h3></td></tr></tbody></table></body></html>");  
            break;
        }
        if (c == '\n') {
          // when starts reading a new line
          blank_line = true;
        }
        else if (c != '\r') {
          // when finds a character on the current line
          blank_line = false;
        }
      }
    }  
    // closing the client connection
    delay(1);
    client.stop();
    Serial.println("Client disconnected.");
  }
}

// only runs once on boot
void setup() {
  // Initializing serial port for debugging purposes
  Serial.begin(115200);
  delay(10);
  
  // Connecting to WiFi network
  setupWifi();

  // Starting the web server
  server.begin();
  Serial.println("Web server running. Waiting for the ESP IP...");
  delay(10000);
  
  //sensors connect
  connectSensor();
  // Printing the ESP IP address
  Serial.println(WiFi.localIP());

  //connect MQTT server
  if (!client_mqtt.connected()) {
    reconnectMqtt();
  }
}

// runs over and over again
void loop() {
  //WiFi server 
  serverTask();

  // If connection to MQTT server was lost, reconnect it 
  if (!client_mqtt.connected()){
    reconnectMqtt();
  } 
  // listen to the selected topic
  client_mqtt.loop();
  
  // send data each 30 minutes 30 x 60 x 1000 = 1800000 milliseconds
  long now = millis();
  if (now - lastMsg > 1800000) {
    lastMsg = now;
    Serial.print("--Send mqtt server message--");
    getWeather();
    dataSend();
  }
  
} 
