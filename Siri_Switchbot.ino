#include <NewPing.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <WiFi.h>
#include <WebServer.h>

#define DHTPIN 14
#define DHTTYPE DHT11

#define TRIGGER_PIN 25
#define ECHO_PIN 33
#define MAX_DISTANCE 200  // Maximum distance to measure (in cm)

#define APPROACH_THRESHOLD 20
#define DEPARTURE_THRESHOLD 24

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Function prototype for reading sensor data
float distance;

// Custom symbol
byte tempSymbol[8] = {
  B00100,
  B01110,
  B01010,
  B01010,
  B01010,
  B10001,
  B10001,
  B01110,
};

byte humSymbol[8] = {
  B00100,
  B00100,
  B01110,
  B01110,
  B11111,
  B11111,
  B11111,
  B01110,
};

// Wi-Fi Credentials
const char* ssid = "LAPTOP";
const char* password = "LMAOOOO6969";
WebServer server(80);

// Set your Static IP address
IPAddress local_IP(192, 168, 137, 29);
// Set your Gateway IP address
IPAddress gateway(192, 168, 137, 1);
IPAddress subnet(255, 255, 255, 0);


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
char options;

float hum;
float temp;

// Variables for Debouncing
unsigned long lastDebounceTime = 0;      // Last time the button state was toggled
const unsigned long debounceDelay = 50;  // Debounce delay in milliseconds

// New Timing Variables for DHT11 and LCD Updates
unsigned long lastDHTUpdate = 0;
unsigned long lastLCDUpdate = 0;
unsigned long lastSonarUpdate = 0;

const unsigned long sonarUpdateInterval = 100;
const unsigned long updateInterval = 2000;  // 2 seconds

// Timing Variables for Gesture Confirmation
unsigned long lastManSwitchUpdate = 0;          // Last time gesture was confirmed
const unsigned long gestureConfirmDelay = 400;  // 1-second delay for gesture confirmation

unsigned long lastToggleTime = 0;           // Time when the last toggle occurred
const unsigned long toggleCooldown = 1000;  // 2-second cooldown period

// State Flags
bool inCooldown = false;        // Indicates if the system is in cooldown
bool gestureConfirmed = false;  // Indicates if a gesture has been confirmed


enum SystemState {
  SIDLE,
  CONFIRMING_GESTURE,
  TOGGLED,
  COOLDOWN
};

SystemState systemState = SIDLE;

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
  lcd.createChar(0, humSymbol);
  lcd.createChar(1, tempSymbol);

  setupWifi();

  // Attach Servos
  s1.attach(s1pin);
  s2.attach(s2pin);
  s1.write(0);
  s2.write(180);

  // Initialize Servos to Neutral Position
  neutralServos();

  Serial.println("Servo Control Initialized.");
}

void loop() {
  server.handleClient();
  handleUltraSonicSensor();
  handleServoState();
  handleDHT();
  handleLCDDisplay('1');
  if (WiFi.status() != WL_CONNECTED) {
    setupWifi();
  }
}


void setupWifi() {
  // Setup Wi-Fi
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("Failed to configure static IP");
  }

  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");

  if (WiFi.status() != WL_CONNECTED) {
    handleDHT();
    handleLCDDisplay('1');

  } else {
    Serial.println("\nConnected to Wi-Fi network with IP Address: ");
    Serial.println(WiFi.localIP());
    lcd.setCursor(0, 0);
    lcd.print("Connected :D    ");
    delay(2000);

    lcd.setCursor(0, 0);
    lcd.print("Hosting Server..");
    delay(2000);
    // Setup web server routes
    server.on("/", handleRoot);
    server.on("/toggle", HTTP_ANY, handleToggle);

    server.begin();
    Serial.println("HTTP server started");
    lcd.clear();
    handleLCDDisplay(3);
  }
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
        s2.write(60);                       // Move Servo 2 to 60 degrees
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
        s1.write(120);                      // Move Servo 1 to 120 degrees
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

void handleLCDDisplay(char options) {

  switch (options) {
    case '1':
      if (millis() - lastLCDUpdate >= updateInterval) {
        lcd.setCursor(0, 1);
        lcd.write(byte(1));
        lcd.print(":");
        lcd.print(temp, 1);
        lcd.write(223);  // Degree symbol
        lcd.print("C ");
        lcd.write(byte(0));
        lcd.print(":");
        lcd.print(hum, 1);
        lcd.print("%");
        lcd.setCursor(0, 0);



        lastLCDUpdate = millis();
      }
      break;
    case '2':
      lcd.setCursor(0, 1);
      lcd.write(byte(1));
      lcd.print(":");
      lcd.print(temp, 1);
      lcd.write(223);  // Degree symbol
      lcd.print("C ");
      lcd.write(byte(0));
      lcd.print(":");
      lcd.print(hum, 1);
      lcd.print("%");
      lcd.setCursor(0, 0);


      break;
    case '3':
      lcd.setCursor(0, 0);
      if (WiFi.status() != WL_CONNECTED) {
        lcd.print("Connecting...   ");
      } else {
        lcd.print("Good Day Wehhh  ");
      }
      break;

    case '4':
      lcd.setCursor(0, 0);
      lcd.print("Hand Detected...");
      break;
    case '5':
      lcd.setCursor(0, 0);
      lcd.print("Cooling Down... ");
      break;
    case '6':
      lcd.setCursor(0, 0);
      lcd.print("Hand Away Bro :)");
      break;
    case '7':
      lcd.setCursor(0, 0);
      lcd.print("Toggle Engaged..");
      break;
    case '8':
      lcd.setCursor(0, 0);
      lcd.print("Toggle Ready... ");
      break;
  }
}

void handleUltraSonicSensor() {
  if (millis() - lastSonarUpdate >= sonarUpdateInterval) {
    distance = sonar.ping_cm();

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    switch (systemState) {
      case SIDLE:
        handleLCDDisplay('3');
        if (distance > 0 && distance <= APPROACH_THRESHOLD) {  // Hand approaching
          handleLCDDisplay('4');                               // Display "Hand Detected..."
          lastManSwitchUpdate = millis();                      // Start confirmation timer
          systemState = CONFIRMING_GESTURE;
        }
        break;

      case CONFIRMING_GESTURE:
        if (distance > DEPARTURE_THRESHOLD) {  // Hand departed during confirmation
          handleLCDDisplay('3');               // Display "Good Day Wehhh"
          systemState = SIDLE;
        } else if (millis() - lastManSwitchUpdate >= gestureConfirmDelay) {  // Gesture confirmed
          handleToggle();
          Serial.println("Gesture confirmed. Toggled state.");
          systemState = TOGGLED;
        }
        break;

      case TOGGLED:
        if (distance > DEPARTURE_THRESHOLD) {  // Hand departed after toggle
          lastToggleTime = millis();           // Start cooldown
          systemState = COOLDOWN;
        } else {
          handleLCDDisplay('6');
        }
        break;

      case COOLDOWN:
        if (distance > 0 && distance <= APPROACH_THRESHOLD) {  // Hand detected during cooldown
          handleLCDDisplay('6');
          lastToggleTime = millis();  // Reset cooldown timer
          Serial.println("Cooldown reset due to new gesture.");
        } else {
          handleLCDDisplay('5');
        }

        if (millis() - lastToggleTime >= toggleCooldown) {  // Cooldown period ended
          handleLCDDisplay('8');
          delay(1000);
          Serial.println("Cooldown period ended. Ready for next gesture.");
          systemState = SIDLE;
        }
        break;
    }

    lastSonarUpdate = millis();
  }
}
