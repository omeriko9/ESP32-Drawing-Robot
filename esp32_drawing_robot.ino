#include <ESP32Servo.h>
#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include "SPIFFS.h"
#include <esp_task_wdt.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoOTA.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <esp_task_wdt.h>


float link1 = 10;  //11.5; 
float link2 = 12.5; //11.5;
float baseWidth = 7.0;

String ssid = "";
String password = "";

#define LSERVO 12 //19 //12 //19 //12 //19
#define RSERVO 13 //18 //13 //18 //13 //18
#define HSERVO 27 //14

// Servo objects
Servo servo1; 
Servo servo2; 
Servo servo3;

// Define the Pair struct
struct Pair {
    float x;
    float y;
};

Pair *imageArray = nullptr; // Pointer to store the large array
size_t arraySize = 0;       // Size of the array

//WebServer server(80); // Simple HTTP server
AsyncWebServer server(80);
size_t receivedSize = 0;    // Total size of data received
size_t totalSize = 0;       // Expected total size of data

int drawDelay = 0;

int penUpAngle = 63;
int penDownAngle = 30;

float t1correction = 0;
float t2correction = 0;

Preferences preferences;

WiFiManager wifiManager;
bool portalUsed = false; // Flag to track if the portal was used

void inline lwrite(float angle)
{
  Serial.print("Moving left servo to angle: " + String(angle) + ", actual angle: " + String(angle+t2correction));
  servo1.write(angle+t1correction);
}

void inline rwrite(float angle)
{
  Serial.print("Moving right servo to angle: " + String(angle) + ", actual angle: " + String(angle+t2correction));
  servo2.write(angle+t2correction);
}


void setupTaskWDT(uint32_t timeout_ms) {
    // Deinitialize any existing WDT first
    esp_task_wdt_deinit();

    // Configure and initialize the WDT
    const esp_task_wdt_config_t twdt_config = {
        .timeout_ms = timeout_ms,
        .idle_core_mask = 0,    // Monitor all cores
        .trigger_panic = true,  // Trigger panic on timeout
    };
    esp_task_wdt_init(&twdt_config);

    // Add the current task to WDT monitoring
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());
}

void HandleOTA()
{

  ArduinoOTA.setPassword("12345");
  ArduinoOTA.setTimeout(60000);

  // Start OTA
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  ArduinoOTA.begin();
}




void saveCredentials(const String &ssid, const String &password) {
    preferences.begin("wifi", false); // Open preferences with namespace "wifi"
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.end();
    Serial.println("WiFi credentials saved to internal memory.");
}

void fetchCredentials(String &ssid, String &password) {
    preferences.begin("wifi", true); // Open preferences in read-only mode
    ssid = preferences.getString("ssid", "");
    password = preferences.getString("password", "");
    preferences.end();
    if (ssid != "" && password != "") {
        Serial.println("WiFi credentials fetched from internal memory.");
    } else {
        Serial.println("No WiFi credentials found in memory.");
    }
}

inline float deg2rad(float deg) {
    return deg * M_PI / 180.0;
}

inline float rad2deg(float rad) {
    return rad * 180.0 / M_PI;
}

// Forward Kinematics (choosing the "upper" intersection above line A'B')
bool forwardKinematics(float theta1_deg, float theta2_deg, float &x, float &y) {
    float t1 = deg2rad(theta1_deg);
    float t2 = deg2rad(theta2_deg);

    // Positions of A' and B'
    float Ax = link1 * cos(t1);
    float Ay = link1 * sin(t1);

    float Bx = baseWidth + link1 * cos(t2);
    float By = link1 * sin(t2);

    float dx = Bx - Ax;
    float dy = By - Ay;
    float d = sqrt(dx*dx + dy*dy);

    // If no intersection
    if (d > 2*link2) return false;

    float Mx = (Ax + Bx)*0.5;
    float My = (Ay + By)*0.5;

    float delta = d/2.0;
    float h = sqrt(link2*link2 - delta*delta);

    // Unit vector along A'->B'
    float ux = dx/d;
    float uy = dy/d;

    // A vector perpendicular to (ux,uy) is (-uy, ux).
    // Upper solution:
    x = Mx - h*uy;
    y = My + h*ux;

    return true;
}

// Inverse Kinematics
bool inverseKinematics(float x, float y, float &theta1_deg, float &theta2_deg) {
    float RA = sqrt(x*x + y*y);
    float RB = sqrt((x - baseWidth)*(x - baseWidth) + y*y);

    if (RA > (link1+link2) || RB > (link1+link2)) return false;

    float cos_g1 = (link1*link1 + RA*RA - link2*link2)/(2*link1*RA);
    float cos_g2 = (link1*link1 + RB*RB - link2*link2)/(2*link1*RB);

    if (fabs(cos_g1) > 1.0 || fabs(cos_g2) > 1.0) return false;

    float g1 = acos(cos_g1);
    float g2 = acos(cos_g2);

    float phiA = atan2(y,x);
    float phiB = atan2(y,x - baseWidth);

    // Upper branch choice:
    // θ1 = φA + g1
    // θ2 = φB - g2
    float theta1 = phiA + g1;
    float theta2 = phiB - g2;

    theta1_deg = rad2deg(theta1);
    theta2_deg = rad2deg(theta2);

    //theta1_deg += t1correction;
    //theta2_deg += t2correction;

    esp_task_wdt_reset();

    return true;
}

void penUp()
{
  for (int i=penDownAngle; i<=penUpAngle; i++)
  {
    servo3.write(i);
    delay(5);
  }
}

void penDown()
{
  for (int i=penUpAngle; i>=penDownAngle; i--)
  {
    servo3.write(i);
    delay(35);
  }
}

void drawLine(float sx, float sy, float ex, float ey, int steps) {
  Serial.print("Starting line drawing, parameters: start x="); Serial.print(sx);
  Serial.print(", start y="); Serial.print(sy);   
  Serial.print(", end x="); Serial.print(ex);  
  Serial.print(", end y="); Serial.print(ey);
  Serial.print(", steps: "); Serial.println(steps);

  penDown();

  // Calculate step increments for x and y
  float xStep = (ex - sx) / steps;
  float yStep = (ey - sy) / steps;

  for (int i = 0; i <= steps; i++) {
    float x = sx + i * xStep; // Calculate current x-coordinate
    float y = sy + i * yStep; // Calculate current y-coordinate

    Serial.print("i = "); Serial.print(i);
    Serial.print(", Moving to: x = "); Serial.print(x);
    Serial.print(", y = "); Serial.print(y);

    float theta1 = 0, theta2 = 0;

    // Calculate inverse kinematics
    if (!inverseKinematics(x, y, theta1, theta2))
    {
        Serial.println("Out of bounds");
        continue;
    }

    Serial.print(", theta1 = ");
    Serial.print(theta1);
    Serial.print(", theta2 = ");
    Serial.println(theta2);

    // Move the servos to the calculated angles    
    lwrite(theta1);    
    rwrite(theta2);

    // Optionally, add a small delay between movements
    delay(30);
  }

  Serial.println("Line drawing completed.");
}

void drawCircle(float cx, float cy, float radius, int steps, bool toMove = true) {
  Serial.print("Starting circle drawing, parameters: x="); Serial.print(cx); Serial.print(", y="); Serial.print(cy); 
  Serial.print(", radius: "); Serial.print(radius); Serial.print(", steps: "); Serial.println(steps);

  float angleStep = 2 * M_PI / steps; // Increment angle by this step size
  for (int i = 0; i <= steps; i++) {
    float angle = i * angleStep; // Current angle
    float x = cx + radius * cos(angle); // Calculate x-coordinate
    float y = cy + radius * sin(angle); // Calculate y-coordinate

    Serial.print("i = "); Serial.print(i); 
    Serial.print(", Current Angle: "); Serial.print(angle); Serial.print(", Moving to: x = ");
    Serial.print(x);
    Serial.print(", y = ");
    Serial.print(y);

    float theta1 = 0,theta2 = 0;

    if (!inverseKinematics(x, y, theta1, theta2))
    {
        Serial.println("Out of bounds");
        continue;
    }

    Serial.print(", theta1 = ");
    Serial.print(theta1);
    Serial.print(", theta2 = ");
    Serial.println(theta2);

    //verifyKinematics(x,y);

    

    if (toMove)
    {
       lwrite(theta1);    
       rwrite(theta2);
    }

    //delay(30);

    // smoothMoveToXY(x, y); // Move to calculated point
  }
  Serial.println("Circle drawing completed.");
}

void drawImage() {
    // Fetch the imageArray from SPIFFS
    

    // Check if imageArray is successfully loaded
    if (imageArray == nullptr || arraySize == 0) {
        Serial.println("No image data to draw.");
        return;
    }

    float theta1, theta2;
    bool penNeedsDown = true;
    


    Serial.println("Starting to draw the image...");

    for (int i = 0; i < arraySize; i++) {
        esp_task_wdt_reset();

        if (imageArray[i].x == -300 && imageArray[i].y == -300) {
            Serial.println("Up Stroke");
            penUp();
            penNeedsDown=true;
            delay(250);
            continue;
        }               

        Serial.print("X="); Serial.print(imageArray[i].x); Serial.print(", Y="); Serial.print(imageArray[i].y);

        // Perform inverse kinematics to calculate angles
        bool res = inverseKinematics(imageArray[i].x, imageArray[i].y, theta1, theta2);
        if (res) {
            // Send angles to the servos
            lwrite(theta1); rwrite(theta2);           

            //Serial.print(", theta1 = "); Serial.print(theta1);
            //Serial.print(", theta2 = "); Serial.println(theta2);
            if (drawDelay > 0)
            {
              delay(drawDelay);
            }
            if (penNeedsDown){
              penDown();            
              penNeedsDown=false;
            }
        } else {
            Serial.println("  Out of bounds");
        }
    }

    // Free the memory allocated for imageArray
    delete[] imageArray;
    imageArray = nullptr;
    arraySize = 0;

    Serial.println("Image drawing complete and memory freed.");
}

void saveCredential(const String &ssid, const String &password) {
    int networkCount = preferences.getInt("networkCount", 0);

    // Save the new SSID and password
    String ssidKey = "ssid" + String(networkCount);
    String passKey = "password" + String(networkCount);

    preferences.putString(ssidKey.c_str(), ssid);
    preferences.putString(passKey.c_str(), password);

    // Update the network count
    preferences.putInt("networkCount", networkCount + 1);

    Serial.println("Credential saved: " + ssid);
}


std::vector<std::pair<String, String>> fetchAllCredentials() {
    std::vector<std::pair<String, String>> networks;

    int networkCount = preferences.getInt("networkCount", 0); // Get the number of saved networks
    for (int i = 0; i < networkCount; i++) {
        String ssidKey = "ssid" + String(i);
        String passKey = "password" + String(i);

        String ssid = preferences.getString(ssidKey.c_str(), "");
        String password = preferences.getString(passKey.c_str(), "");

        if (ssid != "" && password != "") {
            networks.push_back(std::make_pair(ssid, password));
        }
    }

    return networks;
}

void clearAllCredentials() {
    preferences.begin("wifi", false);

    // Get the number of saved networks
    int networkCount = preferences.getInt("networkCount", 0);

    // Remove all stored SSID and password entries
    for (int i = 0; i < networkCount; i++) {
        String ssidKey = "ssid" + String(i);
        String passKey = "password" + String(i);

        preferences.remove(ssidKey.c_str());
        preferences.remove(passKey.c_str());
    }

    // Reset the network count
    preferences.putInt("networkCount", 0);

    preferences.end();

    // Clear Wi-Fi credentials from NVS
    WiFi.disconnect(true);
    delay(100); // Allow time for NVS to clear

    // Reset WiFiManager saved settings
    
    wifiManager.resetSettings();

    nvs_flash_erase();   // Erase all stored NVS data
    nvs_flash_init();    // Reinitialize NVS
    Serial.println("NVS reset completed.");

    Serial.println("All Wi-Fi credentials cleared.");
}

#include "esp_task_wdt.h"

void reenable_task_wdt() {
    // Task Watchdog configuration
    const esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 300000,     // Set a timeout of 5 seconds
        .idle_core_mask = 0,    // Monitor all cores
        .trigger_panic = false, // Don't trigger a panic; just reset
    };

    // Reinitialize the Task Watchdog
    esp_task_wdt_init(&twdt_config);

    // Register the current task for Task Watchdog monitoring
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());

    // Optionally, register other tasks here
}


bool WiFiConnect() {
    preferences.begin("wifi", false);

    // Fetch saved credentials
    std::vector<std::pair<String, String>> networks = fetchAllCredentials();

    // Attempt to connect to each saved network
    for (auto &network : networks) {
        String ssid = network.first;
        String password = network.second;

        if (ssid != "" && password != "") {
            WiFi.begin(ssid.c_str(), password.c_str());
            Serial.println("Connecting to WiFi: " + ssid);
            int attempts = 0;
            while (WiFi.status() != WL_CONNECTED && attempts < 20) {
                delay(500);
                Serial.print(".");
                attempts++;
            }
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("\nWiFi Connected! IP: " + WiFi.localIP().toString());
                reenable_task_wdt();
                return true; // Exit if connected
            } else {
                Serial.println("\nFailed to connect to WiFi: " + ssid);
            }
        }
    }

    Serial.println("No valid WiFi networks connected. Starting captive portal...");

    // Start WiFiManager Captive Portal in non-blocking mode
   

    // Configuration portal callback
    wifiManager.setSaveConfigCallback([]() {
        Serial.println("Configuration saved.");

        // Only restart if successfully connected
        delay(500); // Allow Wi-Fi connection to complete
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("WiFi Configured and Connected. Restarting...");
            ESP.restart();
        } else {
            Serial.println("WiFi credentials saved, but no connection yet.");
        }
    });
  
    // wifiManager.setConfigPortalBlocking(false); // Non-blocking mode
    // wifiManager.setConfigPortalTimeout(300);   // Keep portal open for 5 minutes
    // wifiManager.setAPCallback([](WiFiManager *wm) {
    //     Serial.println("Captive portal is running. Open http://192.168.4.1/");
    // });

     // needed for WifiManager AP Portal
    // esp_task_wdt_deinit();
    wifiManager.autoConnect("Drawing_Robot_Wifi_Setup"); // Start the portal
    return false;
}



void setup() {
    Serial.begin(115200);

    setupTaskWDT(300000);
       
    Serial.println("Free heap memory: " + String(ESP.getFreeHeap()) + " bytes");

    if (!SPIFFS.begin(true)) {
        Serial.println("An error occurred while mounting SPIFFS");
        return;
    }

    Serial.println("SPIFFS mounted successfully.");

    servo1.attach(LSERVO);   
    servo2.attach(RSERVO);   
    servo3.attach(HSERVO);

    //WiFiConnect();  
     
    // Override startConfigPortal to set the portalUsed flag
    wifiManager.setAPCallback([](WiFiManager* wm) {
        portalUsed = true; // Set the flag when the portal starts
        Serial.println("Captive portal started.");
    });

    wifiManager.autoConnect("Drawing_Robot_Wifi_Setup"); // Start the portal
    
    if (portalUsed) {
        Serial.println("Captive portal was used. Restarting ESP...");
        delay(1000);
        ESP.restart(); // Restart the ESP
    } 

    HandleOTA();

    displayHelp(); // Display help instructions on startup

    startServer();
    
}


void displayHelp() {
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("IP Address: " + WiFi.localIP().toString());
    } else {
        Serial.println("Not Connected");
    }
    Serial.println("Commands:");
    Serial.println("- set_l1 <value>: Set link1 to <value>");
    Serial.println("- get_l1: return link1 value");
    Serial.println("- get_wifi: return stored WiFi details");
    Serial.println("- clear_wifi: Clear all WiFi SSIDs and Passwords");
    Serial.println("- set_l2 <value>: Set link2 to <value>");
    Serial.println("- get_l2: return link2 value");
    Serial.println("- set_bw <value>: Set baseWidth to <value>");
    Serial.println("- get_bw: return baseWidth value");
    Serial.println("- get_penup: return Pen up servo3 value");
    Serial.println("- set_penup: Sets Pen up servo3 value");
    Serial.println("- get_pendown: return Pen down servo3 value");
    Serial.println("- set_pendown: Sets Pen down servo3 value");
    Serial.println("- C <x> <y> <radius> <steps>: Draw a circle with specified parameters");
    Serial.println("- LX*<y>: Draw a line parallel to X-axis at y = <y>");
    Serial.println("- LY*<x>: Draw a line parallel to Y-axis at x = <x>");
    Serial.println("- <angle1> <angle2>: Write angles directly to servos");
    Serial.println("- -<t1>,<t2>: Moves servo to angles t1,t2");
    Serial.println("- +<x>,<y>: Move Servos to x,y position");
    Serial.println("- set_ssid <ssid>: Set WiFi SSID");
    Serial.println("- set_pw <password>: Set WiFi password");
    Serial.println("- connect: Connect to WiFi using saved credentials");
    Serial.println("- start_server: Start HTTP server for image array");
    Serial.println("- stop_server: Stop HTTP server");
    Serial.println("- D: Draw the stored image array using drawImage()");
    Serial.println("- Type 'help' to see these instructions again.");
    Serial.println("- fetch_ssid: Print the stored WiFi SSID");
    Serial.println("- fetch_pw: Print the stored WiFi password");
    Serial.println("- U<angle>: Move the 3rd servo (pen up/down) to specific angle");
    Serial.println("- get_t1corr: Get left servo angle correction");
    Serial.println("- get_t2corr: Get right servo angle correction");
    Serial.println("- set_t1corr <angle>: Set left servo angle correction");
    Serial.println("- set_t2corr <angle>: Set right servo angle correction");
    Serial.println("- getfiles: Returns a list of files on the SPIFFS FS");
}

int countOccurrences(const String &str, const String &sub) {
    int count = 0;
    int index = 0;

    while ((index = str.indexOf(sub, index)) != -1) {
        count++;
        index += sub.length();
    }

    return count;
}

void serveIndexPage(AsyncWebServerRequest *request) {

  if (!SPIFFS.begin(true)) {
        request->send(500, "text/plain", "SPIFFS Mount Failed");
        return;
    }

    // Check if the file exists
    if (!SPIFFS.exists("/index.html")) {
        String html = "<h1>Welcome to the Async Web Server</h1>";
        html += "<p>Current heap: " + String(ESP.getFreeHeap()) + " bytes</p>";
        request->send(200, "text/html", html);
        return;
    }

    // Serve the file
    request->send(SPIFFS, "/index.html", "text/html");

    
}


void startServer() {


    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        serveIndexPage(request); // Call the async handler
    });

    // Serve the JavaScript file
    server.serveStatic("/script.js", SPIFFS, "/script.js");

    // Serve the CSS file
    server.serveStatic("/styles.css", SPIFFS, "/styles.css");

    server.on("/penUp", HTTP_GET,[](AsyncWebServerRequest *request) { 
       if (request->hasParam("down") && request->getParam("down")->value() == "true") {
        Serial.println("Pen down request.");
         penDown();
         request->send(200, "text/plain", "OK");

       }
       else
       {       
        Serial.println("Pen up request.");
        request->send(200, "text/plain", "OK");
        penUp();
       }
    });

    server.on("setCorrections", HTTP_GET,[](AsyncWebServerRequest *request) { 
        float lc = request->getParam("lc")->value().toFloat();
        float rc = request->getParam("rc")->value().toFloat();

        Serial.println("Servo correction left=" + String(lc) + ", right =" + String(rc));

        t1correction=lc;
        t2correction=rc;

        request->send(200, "text/plain", "OK");
    });

     server.on("/drawCircle", HTTP_GET,[](AsyncWebServerRequest *request) { 
      
        int x = request->getParam("x")->value().toFloat();
        int y = request->getParam("y")->value().toFloat();
        int r = request->getParam("r")->value().toFloat();
        int s = request->getParam("s")->value().toFloat();
        int lc = request->getParam("lc")->value().toFloat();
        int rc = request->getParam("rc")->value().toFloat();

        Serial.println("Drawing circle x=" + String(x) + ", y=" + String(y) +", r=" + String(r) +", steps: " + String(s) + ", lc: " + String(lc) + ", rc: " + String(rc));
        t1correction=lc;
        t2correction=rc;
        drawCircle(x,y,r,s);
        request->send(200, "text/plain", "OK");

       
    });

    server.on("/draw", HTTP_GET, [](AsyncWebServerRequest *request) {   
        Serial.println(request->url());  // Print the full URL     
        if (request->hasParam("speedDelay")) {
            
            String speedDelayValue = request->getParam("speedDelay")->value();            
            int parsedSpeedDelay = speedDelayValue.toInt();

            if (parsedSpeedDelay >= 0) { 
                drawDelay = parsedSpeedDelay;
                Serial.printf("Speed delay set to: %d\n", drawDelay);
            } else {
                Serial.println("Invalid speedDelay value received, using default.");
            }
        } else {
            Serial.println("No speedDelay parameter provided, using default.");
        }

        if (request->hasParam("penUp")) {            
            String tmpPenUp = request->getParam("penUp")->value();  
            int iPenUp = tmpPenUp.toInt();
            if (iPenUp > 0) {
              penUpAngle = iPenUp;
              Serial.print("Set pen up angle to: ");Serial.println(penUpAngle);
            }
        }

        if (request->hasParam("penDown")) {            
            String tmpPenDown = request->getParam("penDown")->value();  
            int iPenDown = tmpPenDown.toInt();                                  
            if (iPenDown > 0) {
              penDownAngle = iPenDown;
              Serial.print("Set pen down angle to: ");Serial.println(penDownAngle);
            }
        }

        request->send(200, "text/plain", "Drawing Started");
        fetchArrayFromFile(); // Fetch the array and size from SPIFFS           
        Serial.println("Sending array to drawImage...");
        drawImage();
    });


    // Handle POST requests for large data
    server.on("/image_array", HTTP_POST, 
        [](AsyncWebServerRequest *request) {
            // Called when the request is completed
            Serial.printf("Upload complete. Received %zu bytes.\n", receivedSize);

            if (receivedSize % sizeof(Pair) != 0) {
                request->send(400, "text/plain", "Invalid data length.");
            } else {
                size_t pairCount = receivedSize / sizeof(Pair);
                Serial.printf("Received %zu pairs:\n", pairCount);
                for (size_t i = 0; i < pairCount; i++) {
                    Serial.printf("Pair %zu: (%f, %f)\n", i, imageArray[i].x, imageArray[i].y);
                }

                 // Save the array to SPIFFS
                saveArrayToFile(imageArray, pairCount);

                free(imageArray); // Free memory after use
                imageArray = nullptr;
                totalSize = receivedSize = 0;
                request->send(200, "text/plain", "Data received successfully.");
            }
        }
    ).onBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        // This is called during the upload (to process body data)
        if (index == 0) {
            Serial.printf("Receiving %zu bytes...\n", total);
            totalSize = total;
            imageArray = (Pair*)malloc(total); // Allocate memory dynamically
            receivedSize = 0;

            if (!imageArray) {
                Serial.println("Failed to allocate memory.");
                request->send(500, "text/plain", "Server out of memory.");
                return;
            }
        }

        memcpy((uint8_t*)imageArray + index, data, len); // Copy chunk into buffer
        receivedSize += len;

        if (index + len == total) {
            Serial.println("All data received.");
        }
    });

   server.on("/upload2", HTTP_POST, [](AsyncWebServerRequest *request) {
    // This handles the case when the POST request finishes
    request->send(200, "text/plain", "Binary data received successfully");
}, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
    static File uploadFile;

    if (index == 0) {
        // Start of transmission
        Serial.println("Starting binary data transmission...");
        if (!SPIFFS.begin(true)) {
            Serial.println("Failed to initialize SPIFFS");
            request->send(500, "text/plain", "SPIFFS initialization failed");
            return;
        }

        // Open the file for writing
        uploadFile = SPIFFS.open("/array.bin", FILE_WRITE);
        if (!uploadFile) {
            Serial.println("Failed to open /array.bin for writing");
            request->send(500, "text/plain", "Failed to open file for writing");
            return;
        }
    }

    // Write the binary data to the file
    if (uploadFile) {
        size_t bytesWritten = uploadFile.write(data, len);
        if (bytesWritten != len) {
            Serial.println("Error: Not all bytes were written to /array.bin");
            request->send(500, "text/plain", "Failed to write data to file");
            return;
        }
        Serial.printf("Wrote %zu bytes to /array.bin\n", bytesWritten);
    }

    if (index + len == total) {
        // End of transmission
        Serial.println("Binary data transmission completed");
        uploadFile.close();

        // Optionally, verify the file size
        File verifyFile = SPIFFS.open("/array.bin", FILE_READ);
        if (verifyFile) {
            Serial.printf("Final file size: %zu bytes\n", verifyFile.size());
            verifyFile.close();
        } else {
            Serial.println("Failed to open /array.bin for verification");
        }
      }
  });



    server.on(
    "/uploadFile",
    HTTP_POST,
    [](AsyncWebServerRequest *request) {
        // This will be called when the upload is complete
        request->send(200, "text/plain", "File uploaded successfully");
    },
    [](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final) {
        static File uploadFile;

        if (index == 0) {
            // Start of file upload, create or overwrite the file
            Serial.printf("Starting upload of file: %s\n", filename.c_str());
            uploadFile = SPIFFS.open("/" + filename, FILE_WRITE);
            if (!uploadFile) {
                Serial.println("Failed to open file for writing");
                return;
            }
        }

        // Write the current chunk of data to the file
        if (uploadFile) {
            size_t bytesWritten = uploadFile.write(data, len);
            if (bytesWritten != len) {
                Serial.println("Failed to write all data to file");
            }
        }

        // If this is the last chunk, close the file
        if (final) {
            Serial.printf("Finished upload of file: %s\n", filename.c_str());
            if (uploadFile) {
                uploadFile.close();
            }
        }
    });

    server.begin();
    Serial.println("Server started.");
}


void handleUpload(AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final) {
    static File uploadFile;

    if (index == 0) {
        // Start of upload, open the file in FILE_WRITE mode to override
        Serial.println("Starting binary array upload...");
        uploadFile = SPIFFS.open("/array.bin", FILE_WRITE); // Overwrite existing file
        if (!uploadFile) {
            Serial.println("Failed to open array.bin for writing");
            request->send(500, "text/plain", "Failed to open file for writing");
            return;
        }
    }

    // Write the current chunk of data to the file
    if (uploadFile) {
        size_t bytesWritten = uploadFile.write(data, len);
        if (bytesWritten != len) {
            Serial.println("Failed to write all data to array.bin");
            request->send(500, "text/plain", "Failed to write data to file");
            uploadFile.close();
            return;
        }
    }

    // If this is the last chunk, close the file
    if (final) {
        Serial.println("Finished binary array upload");
        if (uploadFile) {
            uploadFile.close();
        }
        request->send(200, "text/plain", "Binary array uploaded and saved successfully");
    }
}



void stopServer()
{
  //server.stop();
  Serial.println("Server cannot be stopped.");
}

void saveArrayToFile(Pair* arr, size_t size) {
    File file = SPIFFS.open("/array.bin", FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }

    // Write the array to the file in binary form
    size_t bytesWritten = file.write((uint8_t*)arr, size * sizeof(Pair));
    if (bytesWritten != size * sizeof(Pair)) {
        Serial.println("Failed to write all data to file");
    } else {
        Serial.printf("Successfully saved %zu bytes to file.\n", bytesWritten);
    }

    file.close();
}

void fetchArrayFromFile() {
    File file = SPIFFS.open("/array.bin", FILE_READ);
    if (!file) {
        Serial.println("Failed to open file for reading");
        return;
    }

    size_t fileSize = file.size();
    if (fileSize % sizeof(Pair) != 0) {
        Serial.println("File size is not a multiple of Pair size, data might be corrupted");
        file.close();
        return;
    }

    size_t pairCount = fileSize / sizeof(Pair);

    // Allocate memory for imageArray
    delete[] imageArray;  // Free any previously allocated memory
    imageArray = new Pair[pairCount];
    arraySize = pairCount;

    // Read the data into imageArray
    size_t bytesRead = file.read((uint8_t*)imageArray, fileSize);
    if (bytesRead != fileSize) {
        Serial.println("Failed to read all data from file");
    } else {
        Serial.printf("Successfully loaded %zu bytes from file.\n", bytesRead);
    }

    file.close();
}

#include "SPIFFS.h"

String listFilesOnSPIFFS() {
    String fileList = ""; // String to hold the file list

    if (!SPIFFS.begin(true)) {
        return "Failed to initialize SPIFFS";
    }

    File root = SPIFFS.open("/");
    if (!root || !root.isDirectory()) {
        return "Failed to open SPIFFS root directory";
    }

    File file = root.openNextFile();
    while (file) {
        // Get file name
        String fileName = file.name();

        // Get file size in KB
        size_t fileSize = file.size();
        size_t fileSizeInKB = fileSize / 1024;

        // Append to list
        fileList += fileName + "," + String(fileSizeInKB) + "KB\r\n";

        file = root.openNextFile(); // Move to next file
    }

    return fileList;
}

float x = 0, y = 0;

void loop() {

    if (WiFi.status() != WL_CONNECTED) {
      
        wifiManager.process(); // Handle captive portal tasks
        delay(1);              // Yield to the system to prevent watchdog resets
        esp_task_wdt_reset();
        return;
    }

    esp_task_wdt_reset();
    ArduinoOTA.handle();

    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        Serial.println("Raw Input: [" + input + "]"); // Debug: Print raw input

        if (input == "D") {
            fetchArrayFromFile(); // Fetch the array and size from SPIFFS
            
            Serial.println("Sending array to drawImage...");
            drawImage();
            
        }
        else if (input.startsWith("set_l1 ")) {
            link1 = input.substring(7).toFloat();
            Serial.println("link1 set to: " + String(link1));
        } 
        else if (input.startsWith("get_l1")) {           
            Serial.println("link1 set to: " + String(link1));
        } 
        else if (input.startsWith("set_l2 ")) {
            link2 = input.substring(7).toFloat();
            Serial.println("link2 set to: " + String(link2));
        } 
        else if (input.startsWith("get_l2")) {            
            Serial.println("link2 set to: " + String(link2));
        } 
        else if (input.startsWith("set_bw ")) {
            baseWidth = input.substring(7).toFloat();
            Serial.println("baseWidth set to: " + String(baseWidth));
        } 
        else if (input.startsWith("get_bw")) {            
            Serial.println("baseWidth set to: " + String(baseWidth));
        } 
        else if (input.startsWith("get_penup")) {            
            Serial.println("Pen up servo angle: " + String(penUpAngle));
        } 
        else if (input.startsWith("set_penup")) {            
            penUpAngle = input.substring(9).toInt();
            Serial.println("Set pen up angle to: " + String(penUpAngle));
        } 
        else if (input.startsWith("get_pendown")) {            
            Serial.println("Pen down servo angle: " + String(penDownAngle));
        } 
        else if (input.startsWith("set_pendown")) {            
            penDownAngle = input.substring(11).toInt();
            Serial.println("Set pen up angle to: " + String(penDownAngle));
        } 
        else if (input.startsWith("get_t1corr"))
        {
            Serial.println("Left servo correction: " + String(t1correction));
        }
        else if (input.startsWith("get_t2corr"))
        {
            Serial.println("Right servo correction: " + String(t2correction));            
        }
        else if (input.startsWith("set_t1corr"))
        {            
            t1correction = input.substring(10).toFloat();
            Serial.println("Left servo correction: " + String(t1correction));
        }
        else if (input.startsWith("set_t2corr"))
        {            
            t2correction = input.substring(10).toFloat();
            Serial.println("Right servo correction: " + String(t2correction));
        }

        else if (input.startsWith("C ")) {
            String params = input.substring(2);            
            int spaceCount = countOccurrences(params, " ");
            
            if (spaceCount == 3) {
                float x = params.substring(0, params.indexOf(' ')).toFloat();
                params = params.substring(params.indexOf(' ') + 1);
                float y = params.substring(0, params.indexOf(' ')).toFloat();
                params = params.substring(params.indexOf(' ') + 1);
                float radius = params.substring(0, params.indexOf(' ')).toFloat();
                float steps = params.substring(params.indexOf(' ') + 1).toFloat();

                drawCircle(x, y, radius, steps);
            } else {
                Serial.println("Error: Invalid number of parameters for drawCircle.");
            }
        } 
        else if (input.startsWith("LX*")) {
            float y = input.substring(3).toFloat();
            Serial.println("Drawing line parallel to X-axis at y = " + String(y));

            float steps = 100;
            
           
            drawLine(-2,y,9,y, steps);
             
            
            
        } 
        else if (input.startsWith("LY*")) {
            float x = input.substring(3).toFloat();
            Serial.println("Drawing line parallel to Y-axis at x = " + String(x));
            
            float steps = 100;
            
            drawLine(x,5,x,12, steps);
             
        } 
        // else if (input.indexOf(' ') > 0) {
        //     float angle1 = input.substring(0, input.indexOf(' ')).toFloat();
        //     float angle2 = input.substring(input.indexOf(' ') + 1).toFloat();            
        //     servo1.write(angle1);
        //     servo2.write(angle2);
        // } 
        else if (input.startsWith("+")) {
            
            float t1, t2;
            float x = input.substring(1, input.indexOf(',')).toFloat();
            float y = input.substring(input.indexOf(',') + 1).toFloat();
            Serial.print("Moving servos to x="); Serial.print(x); Serial.print(", y="); Serial.print(y);

            inverseKinematics(x, y, t1, t2);
            Serial.print(", theta1: ");Serial.print(t1); Serial.print(", theta2: "); Serial.println(t2);
            lwrite(t1); rwrite(t2);
        }     
        else if (input.startsWith("-")) {
                        
            float x = input.substring(1, input.indexOf(',')).toFloat();
            float y = input.substring(input.indexOf(',') + 1).toFloat();
            Serial.print("Moving servos to theta1="); Serial.print(x); Serial.print(", theta2="); Serial.print(y);                        
            lwrite(x); rwrite(y);
        }    
        else if (input.startsWith("U")) {
            float angle = input.substring(1).toFloat();
            Serial.print("Moving servo3 to theta1="); Serial.println(angle);
            servo3.write(angle);

        }   
        
        // else if (input.indexOf('/') > -1) {
        //     //drawCircle(4, 15, 6, 1500);
        //     drawCircle(4, 15, 6, 300, false);
        //     drawCircle(0, 15, 6, 300, false);
        //     drawCircle(8, 15, 6, 300, false);
        // }        
        else if (input.startsWith("set_ssid ")) {
            
            String temp = input.substring(8); // Extract everything after "set_ssid "
            temp.trim(); // Trim leading/trailing whitespace
            if (temp.length() > 0) {
                ssid = temp; // Assign to ssid
                saveCredentials(ssid, password); // Save both SSID and existing password
                Serial.println("SSID set to: [" + ssid + "]");
            } else {
                Serial.println("Error: No SSID provided.");
            }
        }
        else if (input.startsWith("get_wifi")) {
            String s, p;
            Serial.print("From old store method: ");
            fetchCredentials(s, p);
            Serial.print("SSID: "); Serial.print(ssid); Serial.print(", p/w: ");Serial.println(p);

            std::vector<std::pair<String, String>> networks = fetchAllCredentials();
            for (auto &network : networks) {
                String ssid = network.first;
                String password = network.second;

                Serial.print("SSID: ");Serial.print(ssid); Serial.print(", p/w: ");Serial.println(password);
            }
        }
        else if (input.startsWith("clear_wifi")) {
            clearAllCredentials();
            preferences.begin("wifi", false);
            preferences.clear(); // Clear all keys in the "wifi" namespace
            preferences.end();
           
        }
        else if (input.startsWith("set_pw ")) {
            Serial.println("Setting ssid password...");
            String temp = input.substring(7); // Extract everything after "set_pw "
            temp.trim(); // Trim leading/trailing whitespace
            password = temp; // Assign to password
            saveCredentials(ssid, password); // Save both password and existing SSID
            Serial.println("Password set.");
        }      
        else if (input.startsWith("help"))
        {
          displayHelp();
        }
        else if (input == "fetch_ssid") {
            fetchCredentials(ssid, password);
            Serial.println("Stored SSID: " + ssid);
        } 
        else if (input == "fetch_pw") {
            fetchCredentials(ssid, password);
            Serial.println("Stored Password: " + password);
        }
        else if (input == "connect") {
          if (ssid.length() > 0 && password.length() > 0) {
              Serial.println("Connecting to WiFi...");
              WiFi.begin(ssid.c_str(), password.c_str());

              int attempts = 0;
              while (WiFi.status() != WL_CONNECTED && attempts < 2) { // Limit attempts to avoid blocking too long
                  delay(500);
                  Serial.print(".");
                  attempts++;
              }

              if (WiFi.status() == WL_CONNECTED) {
                  Serial.println("\nWiFi connected!");
                  Serial.println("IP Address: " + WiFi.localIP().toString());
              } else {
                  Serial.println("\nFailed to connect to WiFi.");
              }
          } else {
              Serial.println("Error: SSID or Password not set. Use 'set_ssid' and 'set_pw' commands.");
          }
        }
        else if (input == "start_server") {         
          startServer();
        }
        else if (input == "stop_server") {          
          stopServer();
        }
        else if (input == "getfiles") {
           Serial.println(listFilesOnSPIFFS());
        }
        else {
            Serial.println("Invalid command. Type 'help' for instructions.");
        }

    }
}