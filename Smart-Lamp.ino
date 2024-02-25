#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TimeLib.h>
#include <Talkie.h>
#include <FirebaseESP32.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <ESPAsyncWebSrv.h>
#include <ArduinoJson.h>

#define EEPROM_SIZE sizeof(alarms)
const int MAX_REASON_LENGTH = 100;

const char* ssid = "your-ssid";
const char* password = "your-password";

#define FIREBASE_HOST "your-firebase-project.firebaseio.com"
#define FIREBASE_AUTH "your-firebase-auth-token"

AsyncWebServer server(80);
FirebaseData firebaseData;

#define EEPROM_ADDRESS_COLOR 100

struct MyAlarm {
  byte hour;
  byte minute;
  byte second;
  byte day;
  byte month;
  int year;
  char reason[MAX_REASON_LENGTH];
};

const int MAX_ALARMS = 10;
MyAlarm alarms[MAX_ALARMS];
int numAlarms = 0;

struct ColorData {
  int red;
  int green;
  int blue;
};

class LedPin {
private:
  int redPin;
  int greenPin;
  int bluePin;

public:
  LedPin(int rPin, int gPin, int bPin) {
    redPin = rPin;
    greenPin = gPin;
    bluePin = bPin;
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);
  }

  void setColor(const ColorData& color) {
    analogWrite(redPin, color.red);
    analogWrite(greenPin, color.green);
    analogWrite(bluePin, color.blue);
  }
};

const int TriggerPin = 2;
const int Echopin = 3;
float Distance_Threshold = 65;
float ldr_Threshold = 65;
const int ldr = A0;
float value = 0;
float distance_cm = 0;
int x = 0;
int lightcondition = 0;
int color_setter = 0;
const int pushButtonPin = 12;
const int redPin1 = 9;
const int greenPin1 = 10;
const int bluePin1 = 11;
const int redPin2 = 9;
const int greenPin2 = 10;
const int bluePin2 = 11;
const int redPin3 = 9;
const int greenPin3 = 10;
const int bluePin3 = 11;
LiquidCrystal_I2C lcd(0x27, 16, 2);
LedPin rgbLed1(redPin1, greenPin1, bluePin1);
LedPin rgbLed2(redPin2, greenPin2, bluePin2);
LedPin rgbLed3(redPin3, greenPin3, bluePin3);
Talkie voice;
float val = 0;
float dist = 0;
ColorData color;
bool isColorActive;
int globalRedValue = 0;
int globalGreenValue = 0;
int globalBlueValue = 0;
int auto_set = 0; 

void LCD_Display(const char* x) {
  lcd.setCursor(0, 0);
  lcd.print(x);
}

void speechvoice(const uint8_t* x) {
  voice.say(x);
  while (voice.isTalking()) {
    delay(10);
  }
}

bool addAlarm(byte hour, byte minute, byte second, byte day, byte month, int year, const char* reason) {
  if (numAlarms < MAX_ALARMS) {
    alarms[numAlarms].hour = hour;
    alarms[numAlarms].minute = minute;
    alarms[numAlarms].second = second;
    alarms[numAlarms].day = day;
    alarms[numAlarms].month = month;
    alarms[numAlarms].year = year;
    strncpy(alarms[numAlarms].reason, reason, MAX_REASON_LENGTH - 1);
    alarms[numAlarms].reason[MAX_REASON_LENGTH - 1] = '\0';

    numAlarms++;
    return true;
  }
  return false;
}

bool isAlarmTime(const MyAlarm& alarm) {
  byte currentHour = hour();
  byte currentMinute = minute();
  byte currentSecond = second();
  byte currentDay = day();
  byte currentMonth = month();
  int currentYear = year();

  return (currentYear == alarm.year &&
          currentMonth == alarm.month &&
          currentDay == alarm.day &&
          currentHour == alarm.hour &&
          currentMinute == alarm.minute &&
          currentSecond == alarm.second);
}

void triggerAlarm(const MyAlarm& alarm) {
  Serial.print("Alarm triggered: ");
  lcd.print(alarm.reason);
  speechvoice((const uint8_t*)alarm.reason);
  Serial.println(alarm.reason);
  unsigned long startTime = millis();

  while (millis() - startTime < 10000) {
    if (digitalRead(pushButtonPin) == LOW) {
      lcd.clear();
      speechvoice((const uint8_t*)"");
      return;
    }
  }

  lcd.clear();
  speechvoice((const uint8_t*)"");
}

int ldr_value(int ldrPin) {
  int ldrValue = analogRead(ldrPin);
  Serial.print("LDR Value: ");
  Serial.println(ldrValue);
  return ldrValue;
}

int Ultrasonic(int Tpin, int Epin) {
  digitalWrite(Tpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Tpin, LOW);
  float Duration_nanosec = pulseIn(Epin, HIGH);
  float Distance_cm = 0.017 * Duration_nanosec;
  Serial.println(Distance_cm);
  return Distance_cm;
}

int Set_Threshold_Dist(int dist) {
  return dist;
}

int Set_LDR_Threshold(int val) {
  return val;
}

int light_condition(float distance, float value, float val, float dist) {
  if (distance <= dist && value <= val) {
    return 1;
  } else {
    return 0;
  }
}

void checkAlarms() {
  for (int i = 0; i < numAlarms; i++) {
    MyAlarm& alarm = alarms[i];
    if (isAlarmTime(alarm)) {
      triggerAlarm(alarm);
    }
  }
}

void removeAlarm(int index) {
  if (index < 0 || index >= numAlarms) {
    Serial.println("Invalid alarm index");
    return;
  }

  for (int i = index; i < numAlarms - 1; i++) {
    alarms[i] = alarms[i + 1];
  }

  alarms[numAlarms - 1] = MyAlarm();
  numAlarms--;
}

void light_intensity(int x, LedPin& rgb1,LedPin& rgb2,LedPin& rgb3) {
  if (x == 1) {
    ColorData white = {255, 255, 255};
    rgb1.setColor(white);
    rgb2.setColor(white);
    rgb3.setColor(white);
    delay(10);
  }
  if (x == 2) {
    ColorData white = {255, 255, 255};
    rgb1.setColor(white);
    rgb2.setColor(white);
    rgb3.setColor(white);
    delay(10);
  } else {
    ColorData black = {0, 0, 0};
    rgb1.setColor(black);
    delay(10);
  }
}

// Function to handle requests to the "/colour" route
void handleColour(AsyncWebServerRequest *request) {
  if (request->method() == HTTP_POST) {
    // Extract data from URL parameters
    String body=request->arg("r");
    String body1=request->arg("g");
    String body2=request->arg("b");
    globalRedValue = body.toInt();
    globalGreenValue = body1.toInt();
    globalBlueValue = body2.toInt();
    
    // Create a ColorData object and set the color
    ColorData color = {globalRedValue, globalGreenValue, globalBlueValue};
    rgbLed1.setColor(color);
    rgbLed2.setColor(color);
    rgbLed3.setColor(color);
    // Respond to the client
    request->send(200, "application/json", "Received GET data");
  } else {
    request->send(405, "application/json", "405: Method Not Allowed");
  }
}
void handleIntensity(AsyncWebServerRequest *request){
  if (request->method() == HTTP_POST && auto_set == 0) {
    // Extract data from URL parameters
    String body=request->arg("intensity");
    
    int IntensityValue = body.toInt();
    
    // Perform actions based on the received data
    // Perform actions based on the received data
    Serial.print("Received intensity value: ");
    Serial.println(IntensityValue);
    
    // Set the intensity of the LEDs using the global color values
    ColorData intensityColor = {globalRedValue, globalGreenValue, globalBlueValue};
    rgbLed1.setColor(intensityColor);
    rgbLed2.setColor(intensityColor);
    rgbLed3.setColor(intensityColor);
    writeColorToEEPROM();

    // Respond to the client
    request->send(200, "application/json", "Received GET data");
  } else {
    request->send(405, "application/json", "405: Method Not Allowed");
  }
}

void handleAuto(AsyncWebServerRequest *request){
  if (request->method() == HTTP_POST) {
    // Extract data from URL parameters
    String body=request->arg("intensity");
    
    if(body=="on"){
      Serial.println("Auto Mode On");
      auto_set = 1;
    }
    else{
      Serial.println("Auto Mode Off");
      auto_set = 0;
    }
    

    // Respond to the client
    request->send(200, "application/json", "Received GET data");
  } else {
    request->send(405, "application/json", "405: Method Not Allowed");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  EEPROM.begin(EEPROM_SIZE);
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  // Assign a handler function to the "/colour" route for GET requests
  server.on("/colour", HTTP_POST, handleColour);
  server.on("/intensity", HTTP_POST, handleIntensity);
  server.on("/auto", HTTP_POST, handleAuto);

  // Start server
  server.begin();
  readAlarmsFromEEPROM();

  pinMode(redPin1, OUTPUT);
  pinMode(greenPin1, OUTPUT);
  pinMode(bluePin1, OUTPUT);
  pinMode(redPin2, OUTPUT);
  pinMode(greenPin2, OUTPUT);
  pinMode(bluePin2, OUTPUT);
  pinMode(redPin3, OUTPUT);
  pinMode(greenPin3, OUTPUT);
  pinMode(bluePin3, OUTPUT);
  pinMode(TriggerPin, OUTPUT);
  pinMode(Echopin, INPUT);
  pinMode(pushButtonPin, INPUT_PULLUP);
  lcd.init();
  lcd.backlight();
  setTime(12, 34, 56, 21, 8, 2023);
}

void loop() {
  if (Firebase.ready()) {
    readAlarmsFromFirebase();
  } else {
    Serial.println("Firebase is not ready. Using data from EEPROM.");
    readAlarmsFromEEPROM();
  }

  int currentHour = hour();
  int currentMinute = minute();
  int currentSecond = second();
  int currentDay = day();
  int currentMonth = month();
  checkAlarms();
  delay(1000);

  lcd.setCursor(0, 0);
  lcd.print("Time: ");
  if (currentHour < 10) lcd.print("0");
  lcd.print(currentHour);
  lcd.print(":");
  if (currentMinute < 10) lcd.print("0");
  lcd.print(currentMinute);
  lcd.print(":");
  if (currentSecond < 10) lcd.print("0");
  lcd.print(currentSecond);

  lcd.setCursor(0, 1);
  lcd.print("Date: ");
  if (currentMonth < 10) lcd.print("0");
  lcd.print(currentMonth);
  lcd.print("/");
  if (currentDay < 10) lcd.print("0");
  lcd.print(currentDay);

  delay(1000);
  float distance_cm = Ultrasonic(TriggerPin, Echopin);
  float value = ldr_value(ldr);

  int lightcondition = light_condition(distance_cm, value, ldr_Threshold, Distance_Threshold);
  light_intensity(lightcondition, rgbLed1, rgbLed2, rgbLed3);
  delay(1000);
}

void readAlarmsFromFirebase() {
  numAlarms = 0;
  for (int i = 0; i < MAX_ALARMS; i++) {
    String alarmPath = "/alarms/" + String(i) + "/";
    bool isActive = Firebase.getBool(firebaseData, alarmPath + "active");

    if (isActive) {
      alarms[numAlarms].hour = Firebase.getInt(firebaseData, alarmPath + "hour");
      alarms[numAlarms].minute = Firebase.getInt(firebaseData, alarmPath + "minute");
      alarms[numAlarms].second = Firebase.getInt(firebaseData, alarmPath + "second");
      alarms[numAlarms].day = Firebase.getInt(firebaseData, alarmPath + "day");
      alarms[numAlarms].month = Firebase.getInt(firebaseData, alarmPath + "month");
      alarms[numAlarms].year = Firebase.getInt(firebaseData, alarmPath + "year");

      // String reasonStr = Firebase.getString(firebaseData, alarmPath + "reason");
      // strncpy(alarms[numAlarms].reason, reasonStr.c_str(), MAX_REASON_LENGTH - 1);
      // alarms[numAlarms].reason[MAX_REASON_LENGTH - 1] = '\0';

      numAlarms++;
    } else {
      break;
    }
  }
  writeAlarmsToEEPROM();
}


void readAlarmsFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDRESS_COLOR, alarms);
  EEPROM.end();
}

void writeAlarmsToEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(EEPROM_ADDRESS_COLOR, alarms);
  EEPROM.commit();
  EEPROM.end();
}


void readColorFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDRESS_COLOR, color.red);   // Read the red component
  EEPROM.get(EEPROM_ADDRESS_COLOR + sizeof(int), color.green); // Read the green component
  EEPROM.get(EEPROM_ADDRESS_COLOR + 2 * sizeof(int), color.blue);  // Read the blue component
  EEPROM.end();
}


void writeColorToEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(EEPROM_ADDRESS_COLOR, color.red);   // Store red component
  EEPROM.put(EEPROM_ADDRESS_COLOR + sizeof(int), color.green); // Store green component
  EEPROM.put(EEPROM_ADDRESS_COLOR + 2 * sizeof(int), color.blue);  // Store blue component
  EEPROM.commit();
  EEPROM.end();
}