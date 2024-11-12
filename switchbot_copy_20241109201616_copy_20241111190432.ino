#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <WiFi.h>
#include <WebServer.h>

#define DHTPIN 14
#define DHTTYPE DHT11

// Wi-Fi Credentials
const char* ssid = "LAPTOP";
const char* password = "LMAOOOO6969";
WebServer server(80);

// Toggle State
bool toggleState = false;

// Servo Objects
Servo s1;
Servo s2;

// Pin Assignments
const int s1pin = 27;
const int s2pin = 26;

LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHTPIN, DHTTYPE);

float hum;
float temp;

// Variables for Debouncing
unsigned long lastDebounceTime = 0;      // Last time the button state was toggled
const unsigned long debounceDelay = 50;  // Debounce delay in milliseconds

// New Timing Variables for DHT11 and LCD Updates
unsigned long lastDHTUpdate = 0;
unsigned long lastLCDUpdate = 0;
const unsigned long updateInterval = 2000;  // 2 seconds

// Servo Control States
enum ServoState {
  IDLE,
  OPENING_S1,
  OPENING_S2,
  CLOSING_S2,
  CLOSING_S1,
  RETURNING_TO_NEUTRAL  // Added new state
};

ServoState servoState = IDLE;

// Timing Variables
unsigned long actionStartTime = 0;
const unsigned long actionDelay = 500;  // 500ms delay between servo actions

void setup() {
  Serial.begin(115200);

  dht.begin();
  // Initialize the LCD
  lcd.init();
  lcd.backlight();

  // Setup Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi network with IP Address: ");
  Serial.println(WiFi.localIP());

  // Setup web server routes
  server.on("/", handleRoot);
  server.on("/toggle", HTTP_ANY, handleToggle);

  server.begin();
  Serial.println("HTTP server started");

  // Attach Servos
  s1.attach(s1pin);
  s2.attach(s2pin);

  // Initialize Servos to Neutral Position
  neutralServos();

  Serial.println("Servo Control Initialized.");
}

void loop() {
  server.handleClient();
  handleServoState();
  handleDHT();
  handleLCDDisplay();
}

void handleRoot() {
  // Serve a simple web page with a button to toggle the state
  String html = "<html><body><h1>Servo Control</h1>";
  html += "<p>Toggle State: " + String(toggleState ? "ON" : "OFF") + "</p>";
  html += "<form action=\"/toggle\" method=\"POST\"><input type=\"submit\" value=\"Toggle\"></form>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleToggle() {
  toggleState = !toggleState;

  // Print the new state to the Serial Monitor
  Serial.print("Toggle State: ");
  Serial.println(toggleState ? "ON" : "OFF");

  // Set Servo State based on toggle
  if (toggleState) {
    servoState = OPENING_S1;
    actionStartTime = millis();
  } else {
    servoState = CLOSING_S2;  // Start closing with Servo 2
    actionStartTime = millis();
  }

  // Redirect back to the root page
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

//////////////////////////
// Servo State Handling //
//////////////////////////

void handleServoState() {
  switch (servoState) {
    case IDLE:
      // Do nothing
      break;

    case OPENING_S1:
      if (millis() - actionStartTime >= actionDelay) {
        Serial.println("Opening Servo 1...");
        s1.write(0);  // Move Servo 1 to 0 degrees
        servoState = OPENING_S2;
        actionStartTime = millis();
      }
      break;

    case OPENING_S2:
      if (millis() - actionStartTime >= actionDelay) {
        Serial.println("Opening Servo 2...");
        s2.write(60);  // Move Servo 2 to 60 degrees
        servoState = RETURNING_TO_NEUTRAL;  // Transition to neutral state
        actionStartTime = millis();
        Serial.println("Servos Opened.");
      }
      break;

    case CLOSING_S2:
      if (millis() - actionStartTime >= actionDelay) {
        Serial.println("Closing Servo 2...");
        s2.write(180);  // Move Servo 2 to 180 degrees
        servoState = CLOSING_S1;
        actionStartTime = millis();
      }
      break;

    case CLOSING_S1:
      if (millis() - actionStartTime >= actionDelay) {
        Serial.println("Closing Servo 1...");
        s1.write(120);  // Move Servo 1 to 120 degrees
        servoState = RETURNING_TO_NEUTRAL;  // Transition to neutral state
        actionStartTime = millis();
        Serial.println("Servos Closed.");
      }
      break;

    case RETURNING_TO_NEUTRAL:
      if (millis() - actionStartTime >= actionDelay) {
        Serial.println("Returning Servos to Neutral...");
        neutralServos();  // Return servos to neutral position
        servoState = IDLE;
      }
      break;
  }
}

///////////////////////
// Servo Control Logic //
///////////////////////

void neutralServos() {
  s1.write(0);
  s2.write(180);
  Serial.println("Servos set to Neutral.");
}

void handleDHT() {
  if (millis() - lastDHTUpdate >= updateInterval) {
    float newHum = dht.readHumidity();
    float newTemp = dht.readTemperature();

    if (!isnan(newHum) && !isnan(newTemp)) {
      hum = newHum;
      temp = newTemp;
      Serial.print("Humidity: ");
      Serial.print(hum);
      Serial.print("%, Temperature: ");
      Serial.print(temp);
      Serial.println("C");
    } else {
      Serial.println("Failed to read from DHT sensor!");
    }
    lastDHTUpdate = millis();
  }
}

void handleLCDDisplay() {
  if (millis() - lastLCDUpdate >= updateInterval) {
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temp);
    lcd.write(223);    // Degree symbol
    lcd.print("C  ");  // Extra spaces to clear old data

    lcd.setCursor(0, 1);
    lcd.print("Humi: ");
    lcd.print(hum);
    lcd.print("%   ");  // Extra spaces to clear old data

    lastLCDUpdate = millis();
  }
}
