#include "HardwareSerial.h"
#include <HTTPClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <math.h>

#define JSON_BUFFER_SIZE 16384  // Increased buffer size
#define GPS_FIX_TIMEOUT 60000   // 60 seconds timeout for GPS fix
#define MIN_SATELLITES 3        // Minimum satellites for reliable fix

// Variable declarations
float kgain;
int counter = 0;
int tx = 25;
int rx = 26;
const int STM_TX = 16;  // ESP32 IO16 -> STM32 RX
const int STM_RX = 17;  // ESP32 IO17 -> STM32 TX
const int adc6 = 34;
float variance = -1;
float minAccuracy = 1.0;
float Q = 3.0;
float dt;
String time1;
String hour1, hour2, minute1, minute2, second1, second2;
float ms;
float accuracy = 2.0;
float lat1 = 0, lat2 = 0, long1 = 0, long2 = 0;

// Add these missing declarations
String utc;
String latitude1;
String longitude1;
String run1;
String fix1;
String msla;
String otg;
String cog;
String fixmode;
String hdop;
String pdop;
String vdop;
String sat1;
String sat2;
String sat3;
String cno;
String hpa;
String vpa;

String response1 = "";
String data1[21];
String old[21];
int m;
int Powerkey = 13;

#define DEBUG true
HardwareSerial stm32Serial(1); 
HardwareSerial sim808(2);

struct NavigationState {
    String destination;
    bool isNavigating;
    int currentStep;
    JsonArray steps;
    float nextStepLat;
    float nextStepLong;
    float totalDistance;
    float distanceToNextStep;
} navState;

//const char* ssid = "Tselhome-A468";
//const char* password = "53160617";
const char* ssid = "Tselhome-A468";
const char* password = "53160617";

String HOST_NAME = "https://arifin.tech/esp32";
String PATH_NAME = "/esp_temp.php";

const String API_KEY = "AIzaSyBkVQ0GGqeHPWmDZgRAx2aMmubRXesoJrI";
void power() {
    Serial.println("Powering SIM808...");
    digitalWrite(Powerkey, LOW);
    delay(5000);
    digitalWrite(Powerkey, HIGH);
    Serial.println("Power sequence completed");
}

void getgps() {
    Serial.println("Initializing GPS...");
    sendDatainit("AT+CGNSPWR=1", 1000, DEBUG);
    sendDatainit("AT+CGNSSEQ=RMC", 1000, DEBUG);
}

String sendData(String command, const int timeout, boolean debug) {
    Serial.println("Sending command: " + command);
    response1 = "";
    
    sim808.println(command);
    long int time = millis();
    
    while ((time + timeout) > millis()) {
        while (sim808.available()) {
            char c = sim808.read();
            response1 += c;
        }
    }
    
    if (debug) {
        // Find the actual response data after "+CGNSINF: "
        int startPos = response1.indexOf("+CGNSINF: ");
        if (startPos >= 0) {
            String gpsData = response1.substring(startPos + 9); // Skip "+CGNSINF: "
            
            // Parse the comma-separated values
            int i = 0;
            int k = 0;
            data1[k] = "";
            
            while (i < gpsData.length() && k < 21) {
                if (gpsData[i] != ',') {
                    data1[k] += gpsData[i];
                } else {
                    k++;
                    data1[k] = "";
                }
                i++;
            }
            
            // Extract the values from the correct positions in the GNSINF response
            utc = data1[2];
            latitude1 = data1[3];
            longitude1 = data1[4];
            fix1 = data1[1];        
            sat1 = data1[15];       
            
            // Print parsed data with all relevant information
            Serial.println("\nGPS Status:");
            Serial.println("Run Status: " + data1[0] + " (1=running)");  
            Serial.println("Fix Status: " + data1[1] + " (1=fix acquired)");
            Serial.println("UTC Time: " + data1[2]);
            Serial.println("Latitude: " + data1[3]);
            Serial.println("Longitude: " + data1[4]);
            Serial.println("Altitude: " + data1[5] + " meters");
            Serial.println("Ground Speed: " + data1[6] + " km/h");
            Serial.println("Course: " + data1[7] + " degrees");
            Serial.println("Satellites Used: " + data1[15]);
            Serial.println("Fix Quality (HDOP): " + data1[10]);
        }
    }
    return response1;
}

String sendDatainit(String command, const int timeout, boolean debug) {
    String response = "";
    Serial.println("Sending init command: " + command);
    
    sim808.println(command);
    long int time = millis();
    
    while ((time + timeout) > millis()) {
        while (sim808.available()) {
            char c = sim808.read();
            response += c;
        }
    }
    
    if (debug) {
        Serial.println("Init Response: " + response);
    }
    return response;
}
String getDestinationFromUser() {
    Serial.println("Enter destination name:");
    Serial.println("(Type your destination and press Enter)");
    
    // Clear any existing serial data
    while(Serial.available()) {
        Serial.read();
    }
    
    // Wait for new input with less frequent GPS updates
    unsigned long lastUpdate = 0;
    while (!Serial.available()) {
        unsigned long currentMillis = millis();
        if (currentMillis - lastUpdate >= 5000) {  
            sendData("AT+CGNSINF", 300, DEBUG);
            lastUpdate = currentMillis;
        }
        delay(100); 
    }
    
    String destination = Serial.readStringUntil('\n');
    destination.trim();
    
    destination.replace(" ", "%20");
    destination.replace(",", "%2C");
    
    Serial.println("Received destination: " + destination);
    return destination;
}


// Calculate distance between two points using Haversine formula
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    float R = 6371000; // Earth's radius in meters
    float phi1 = lat1 * PI / 180;
    float phi2 = lat2 * PI / 180;
    float deltaPhi = (lat2 - lat1) * PI / 180;
    float deltaLambda = (lon2 - lon1) * PI / 180;

    float a = sin(deltaPhi/2) * sin(deltaPhi/2) +
              cos(phi1) * cos(phi2) *
              sin(deltaLambda/2) * sin(deltaLambda/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

bool hasGPSFix() {
    return (fix1 == "1");
}

// Extract coordinates from Google Maps encoded polyline
bool getStepCoordinates(JsonVariant step, float& lat, float& lng) {
    if (step.containsKey("end_location")) {
        lat = step["end_location"]["lat"].as<float>();
        lng = step["end_location"]["lng"].as<float>();
        return true;
    }
    return false;
}

String getDirections(float origin_lat, float origin_lng, String destination_name) {
    // Add validation and debug output
    Serial.println("\nValidating coordinates:");
    Serial.println("Origin Latitude: " + String(origin_lat, 6));
    Serial.println("Origin Longitude: " + String(origin_lng, 6));
    
    if (origin_lat == 0 || origin_lng == 0) {
        Serial.println("Error: Invalid origin coordinates");
        return "";
    }

    // Verify the coordinates are reasonable for Indonesia
    if (origin_lat < -11 || origin_lat > 6 || origin_lng < 95 || origin_lng > 141) {
        Serial.println("Error: Coordinates outside Indonesia");
        return "";
    }

    Serial.println("\nGetting directions from: " + String(origin_lat, 6) + "," + String(origin_lng, 6));
    Serial.println("To destination: " + destination_name);

    String url = "https://maps.googleapis.com/maps/api/directions/json";
    url += "?origin=" + String(origin_lat, 6) + "," + String(origin_lng, 6);
    url += "&destination=" + destination_name;
    url += "&key=" + API_KEY;

    Serial.println("Making request to: " + url);

    HTTPClient http;
    http.begin(url);

    int httpCode = http.GET();
    Serial.println("HTTP Response code: " + String(httpCode));

    if (httpCode > 0) {
        String payload = http.getString();
        Serial.println("Response size: " + String(payload.length()) + " bytes");
        if (payload.length() > 0) {
            Serial.println("First 100 characters of response: " + payload.substring(0, 100));
        }
        http.end();
        return payload;
    } else {
        Serial.println("Error getting directions: " + http.errorToString(httpCode));
        http.end();
        return "";
    }
}

void updateNavigation(float current_lat, float current_lng) {
    if (!navState.isNavigating) return;

    // Calculate distance to next step
    navState.distanceToNextStep = calculateDistance(
        current_lat, current_lng,
        navState.nextStepLat, navState.nextStepLong
    );

    // Display current navigation info
    Serial.println("\nCurrent Navigation Status:");
    Serial.print("Distance to next turn: ");
    Serial.print(navState.distanceToNextStep);
    Serial.println(" meters");

    // Check if we've reached the next step (within 20 meters)
    if (navState.distanceToNextStep < 20) {
        navState.currentStep++;
        
        // Check if we've reached the destination
        if (navState.currentStep >= navState.steps.size()) {
            Serial.println("You have reached your destination!");
            navState.isNavigating = false;
            return;
        }

        // Get next step coordinates
        JsonVariant nextStep = navState.steps[navState.currentStep];
        if (getStepCoordinates(nextStep, navState.nextStepLat, navState.nextStepLong)) {
            // Display new instruction
            String instruction = nextStep["html_instructions"].as<String>();
            float distance = nextStep["distance"]["value"].as<float>();
            float duration = nextStep["duration"]["value"].as<float>();

            // Clean up instruction text
            instruction.replace("<b>", "");
            instruction.replace("</b>", "");
            instruction.replace("<div style=\"font-size:0.9em\">", "\n  ");
            instruction.replace("</div>", "");

            Serial.println("\nNew Instruction:");
            Serial.println(instruction);
            Serial.print("Distance: ");
            Serial.print(distance / 1000.0, 2);
            Serial.println(" km");
            Serial.print("Duration: ");
            Serial.print(duration / 60.0, 1);
            Serial.println(" minutes");
        }
    }

    // Check if we need to recalculate route (if too far from expected path)
    float threshold = 50.0; // meters
    if (navState.distanceToNextStep > threshold) {
        Serial.println("Recalculating route...");
        String directions = getDirections(current_lat, current_lng, navState.destination);
        
        if (directions.length() > 0) {
            DynamicJsonDocument doc(JSON_BUFFER_SIZE);
            DeserializationError error = deserializeJson(doc, directions);
            
            if (!error) {
                JsonArray routes = doc["routes"];
                if (routes.size() > 0) {
                    navState.steps = routes[0]["legs"][0]["steps"];
                    navState.currentStep = 0;
                    
                    if (getStepCoordinates(navState.steps[0], navState.nextStepLat, navState.nextStepLong)) {
                        Serial.println("Route recalculated successfully");
                    }
                }
            }
        }
    }
}

void startNavigation(String destination_name, float current_lat, float current_lng) {
    // Reset navigation state
    navState.isNavigating = false;
    navState.currentStep = 0;
    navState.distanceToNextStep = 0;
    
    Serial.println("Starting navigation to: " + destination_name);
    
    String directions = getDirections(current_lat, current_lng, destination_name);
    if (directions.length() > 0) {
        DynamicJsonDocument doc(JSON_BUFFER_SIZE);  // Larger buffer
        DeserializationError error = deserializeJson(doc, directions);
        
        if (error) {
            Serial.println("Failed to parse JSON: " + String(error.c_str()));
            return;
        }
        
        JsonArray routes = doc["routes"];
        if (routes.size() == 0) {
            Serial.println("No routes found!");
            return;
        }

        // Initialize navigation state
        navState.destination = destination_name;
        navState.isNavigating = true;
        navState.currentStep = 0;
        navState.steps = routes[0]["legs"][0]["steps"];
        
        if (getStepCoordinates(navState.steps[0], navState.nextStepLat, navState.nextStepLong)) {
            // Display initial instruction
            JsonVariant firstStep = navState.steps[0];
            String instruction = firstStep["html_instructions"].as<String>();
            float distance = firstStep["distance"]["value"].as<float>();
            float duration = firstStep["duration"]["value"].as<float>();

            instruction.replace("<b>", "");
            instruction.replace("</b>", "");
            instruction.replace("<div style=\"font-size:0.9em\">", "\n  ");
            instruction.replace("</div>", "");

            Serial.println("\nFirst Instruction:");
            Serial.println(instruction);
            Serial.print("Distance: ");
            Serial.print(distance / 1000.0, 2);
            Serial.println(" km");
            Serial.print("Duration: ");
            Serial.print(duration / 60.0, 1);
            Serial.println(" minutes");
        }
    }
}
void setup() {
    Serial.begin(9600);
    delay(1000);
    Serial.println("\nESP32 GPS Logger Starting...");
    
    pinMode(Powerkey, OUTPUT);
    Serial.println("Power key initialized");
    
    power();
    
    sim808.begin(9600, SERIAL_8N1, rx, tx);
    Serial.println("SIM808 Serial initialized");
    
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected!");
    Serial.println("IP address: " + WiFi.localIP().toString());
    
    getgps();
    delay(1000);

    stm32Serial.begin(115200, SERIAL_8N1, STM_RX, STM_TX);
    Serial.println("STM32 Serial initialized");
}

void loop() {
    counter++;
    Serial.println("\n--- Loop " + String(counter) + " ---");
    
    // Check for serial input first, before any GPS operations
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.equals("new") || input.equals("new destination")) {
            // Clear any remaining data in the serial buffer
            while(Serial.available()) {
                Serial.read();
            }
            
            String destination_name = getDestinationFromUser();
            if (destination_name.length() > 0 && hasGPSFix()) {
                startNavigation(destination_name, lat1, long1);
            } else if (!hasGPSFix()) {
                Serial.println("Cannot start navigation - No GPS fix");
            }
            
            // Clear any remaining data after getting destination
            while(Serial.available()) {
                Serial.read();
            }
        } else if (input.equals("stop")) {
            navState.isNavigating = false;
            Serial.println("Navigation stopped");
        }
    }
    
    // Regular GPS and navigation updates
    sendDatainit("AT+CGNSPWR=1", 2800, DEBUG);
    sendData("AT+CGNSINF", 300, DEBUG);
    
    if (data1[2].length() >= 14) {
        hour2 = data1[2].substring(8, 10);
        minute2 = data1[2].substring(10, 12);
        second2 = data1[2].substring(12, 14);
        
        Serial.println("Time parsed - Hour: " + hour2 + " Minute: " + minute2 + " Second: " + second2);
    } else {
        Serial.println("Warning: Invalid time data received");
    }
    
    if (counter != 1) {
        ms = (hour2.toFloat() - hour1.toFloat()) * 60 * 60 * 1000 +
             (minute2.toFloat() - minute1.toFloat()) * 60 * 1000 +
             (second2.toFloat() - second1.toFloat()) * 1000;
        
        Serial.println("Time difference (ms): " + String(ms));
        
        if (accuracy < minAccuracy) accuracy = minAccuracy;
        
        if (variance < 0) {
            lat2 = data1[3].toFloat();
            long2 = data1[4].toFloat();
            variance = accuracy * accuracy;
            Serial.println("Initial position set");
        } else {
            lat2 = data1[3].toFloat();
            long2 = data1[4].toFloat();
            variance += ms * Q * Q / 1000;
            kgain = variance / (variance + accuracy * accuracy);
            lat1 += kgain * (lat2 - lat1);
            long1 += kgain * (long2 - long1);
            variance = (1 - kgain) * variance;
            
            Serial.println("Kalman filtered coordinates:");
            Serial.println("Latitude: " + String(lat1, 6));
            Serial.println("Longitude: " + String(long1, 6));
        }
    }
    
    hour1 = hour2;
    minute1 = minute2;
    second1 = second2;
    
    latitude1 = String(lat1, 6);
    longitude1 = String(long1, 6);
    
    String postData = "temperature=" + latitude1 + "&long1=" + longitude1 + "&id=" + utc;
    Serial.println("Posting data: " + postData);
    
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(HOST_NAME + PATH_NAME);
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");
        
        auto httpCode = http.POST(postData);
        String payload = http.getString();
        
        Serial.println("HTTP Response code: " + String(httpCode));
        Serial.println("Server response: " + payload);
        
        http.end();
    } else {
        Serial.println("WiFi connection lost!");
    }

    // Update navigation if active and we have GPS fix
    if (navState.isNavigating && hasGPSFix()) {
        updateNavigation(lat1, long1);
    }
if (navState.isNavigating && hasGPSFix()) {
        String stm32Data = "Latitude: " + String(lat1, 6) + "\n"
                        + "Longitude: " + String(long1, 6) + "\n";
        
        // Add current navigation instruction if available
        if (navState.currentStep < navState.steps.size()) {
            JsonVariant currentStep = navState.steps[navState.currentStep];
            String instruction = currentStep["html_instructions"].as<String>();
            float distance = currentStep["distance"]["value"].as<float>();
            float duration = currentStep["duration"]["value"].as<float>();

            instruction.replace("<b>", "");
            instruction.replace("</b>", "");
            instruction.replace("<div style=\"font-size:0.9em\">", "\n  ");
            instruction.replace("</div>", "");

            stm32Data += "First Instruction: " + instruction + "\n"
                      + "Distance: " + String(distance / 1000.0, 2) + " km\n"
                      + "Duration: " + String(duration / 60.0, 1) + " minutes\n"
                      + "Current Navigation Status:\n"
                      + "Distance to next turn: " + String(navState.distanceToNextStep) + " meters\n";
        }
        
        // Send to STM32
        stm32Serial.print(stm32Data);
    }
    delay(2000);
}