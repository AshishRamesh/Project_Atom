#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>

const char* ssid = "POCO_M2_Pro";          // Replace with your Wi-Fi SSID
const char* password = "ashish09";   // Replace with your Wi-Fi password

ESP8266WebServer server(8080);
Servo myServo;

// This function will handle angle update requests
void handleUpdate() {
  if (server.hasArg("angle")) {
    int angle = server.arg("angle").toInt();
    
    // Constrain angle to avoid servo over-rotation
    angle = constrain(angle, 0, 180);

    // Set servo position
    myServo.write(angle);
    server.send(200, "text/plain", "Angle updated");
  } else {
    server.send(400, "text/plain", "Bad Request: No angle provided");
  }
}

void setup() {
  Serial.begin(115200);
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.print("ESP8266 IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize the servo on GPIO 5 (D1)
  myServo.attach(5);
  myServo.write(90); // Start at 90 degrees

  // Set up the server to handle the /update endpoint
  server.on("/update", handleUpdate);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  // Handle client requests
  server.handleClient();
}
