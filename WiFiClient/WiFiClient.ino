#include <WiFi.h>

//telnet 192.168.8.194 23

const char* ssid = "Hi Wifi";
const char* password = "Neme9s8i7s";

WiFiServer server(23);  // Telnet port

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    while (client.connected()) {
      // Your regular loop code here
      // Use client.println() instead of Serial.println() for debug messages
      client.println("Debug message");
      delay(1000);
    }
  }
}