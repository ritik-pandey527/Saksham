#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_task_wdt.h> // For watchdog

// =================== CONFIGURATION ===================
const char* ssid = "your_ssid";         // WiFi SSID
const char* password = "your_password"; // WiFi Password
const char* mdnsName = "wheelchair";    // http://wheelchair.local/

// Motor Pins
const int LM_IN1 = 23;
const int LM_IN2 = 22;
const int RM_IN3 = 21;
const int RM_IN4 = 19;

// Hall Effect Sensor Pins
const int hallSensor1 = 25;
const int hallSensor2 = 27;
const int hallSensor3 = 26;

// Joystick Pins
#define yAxis_pin 35
#define xAxis_pin 34
#define button_pin 32

// Buzzer and Status LED
#define buzzer_pin 13
#define status_led 2 // Onboard LED

// Sequence Tracking (for path recording/replay)
struct Movement {
  int type;      // 1=Forward, 2=Left, 3=Right, 5=Stop
  int duration;  // ms
};
Movement* seq_Array;
int seq = 0;
int seqArraySize = 1000;

// Mode and State
String mode = "joystick";
String lastCommand = "Stopped";
bool hornActive = false;
bool isRecording = false;
bool isReplaying = false;
bool isReversing = false;
char lastMovement = 'S';

// Timing and Debounce
unsigned long moveStartTime = 0;
unsigned long buttonPressStart = 0;
unsigned long buttonPressDuration = 0;
unsigned long lastModeChange = 0;
unsigned long lastCmdTime = 0;
const unsigned long modeChangeCooldown = 1000; // ms
const unsigned long debounceMs = 200; // Debounce time for commands

// Non-blocking Buzz
unsigned long buzzStart = 0;
int buzzDuration = 0;
bool buzzing = false;

// Non-blocking Replay
bool replaying = false;
int replayIndex = 0;
unsigned long replayMoveStart = 0;
bool replayReverse = false;

// Web Server
WebServer server(80);

// =================== BATTERY STATUS (Dummy) ===================
// Replace with actual ADC read if available
int getBatteryPercent() {
  // Example: return analogRead(batteryPin) * 100 / 4095;
  return 85;
}

// =================== HTML HELPERS ===================
String getStatusBar() {
  String wifi = (WiFi.status() == WL_CONNECTED) ? "Connected" : "Disconnected";
  String ip = WiFi.localIP().toString();
  int battery = getBatteryPercent();
  return "<div style='font-size:1.1em;margin-bottom:10px;'>"
         "Mode: <b>" + mode + "</b> | WiFi: <b>" + wifi + "</b> | IP: <b>" + ip + "</b> | Battery: <b>" + String(battery) + "%</b>"
         "</div>";
}

// =================== HTML PAGES ===================

// Mode selection UI (with button to car control)
const char* mode_webpage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Wheelchair Mode Selection</title>
    <style>
        body { font-family: Arial, sans-serif; display: flex; justify-content: center; align-items: center; height: 100vh; background-color: #f4f4f4; margin: 0; }
        .container { background: white; padding: 30px; border-radius: 12px; box-shadow: 0 4px 10px rgba(0, 0, 0, 0.1); text-align: center; max-width: 350px; width: 100%; }
        h2 { color: #333; margin-bottom: 20px; }
        .eye { background-color: #ff9800; color: white; }
        .eye:hover { background-color: #e68900; }
        button { font-size: 22px; padding: 18px; margin: 12px; width: 100%; border: none; border-radius: 8px; cursor: pointer; transition: 0.3s; }
        .joystick { background-color: #007bff; color: white; }
        .joystick:hover { background-color: #0056b3; }
        .hall { background-color: #28a745; color: white; }
        .hall:hover { background-color: #1e7e34; }
        .car { background-color: #222; color: #fff; }
        .car:hover { background-color: #444; }
        #status { margin-top: 20px; font-weight: bold; color: #555; }
        .home { background: #555; color: #fff; margin-top: 10px;}
        .home:hover { background: #222; }
    </style>
    <script>
        function setMode(mode) {
            fetch('/setMode?mode=' + mode)
                .then(response => response.text())
                .then(data => {
                    document.getElementById('status').innerText = "Mode: " + data;
                    updateStatusBar();
                });
        }
        function gotoCar() {
            window.location.href = "/car";
        }
        function updateStatusBar() {
            fetch('/status').then(r=>r.text()).then(html=>{document.getElementById('statusbar').innerHTML=html;});
        }
        window.onload = updateStatusBar;
        setInterval(updateStatusBar, 5000);
    </script>
</head>
<body>
    <div class="container">
        <div id="statusbar"></div>
        <h2>Select Control Mode</h2>
        <button class="joystick" onclick="setMode('joystick')">Joystick Mode</button>
        <button class="hall" onclick="setMode('hall')">Hall Effect Mode</button>
        <button class="eye" onclick="setMode('eye')">Eye Control Mode</button>
        <button class="car" onclick="gotoCar()">Go to Wheelchair Car Control</button>
        <h3 id="status">Mode: Unknown</h3>
    </div>
</body>
</html>
)rawliteral";

// Car-style control UI (with button to mode selection, horn, feedback, home)
const char* car_webpage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Wheelchair Car Control</title>
  <style>
    body { font-family: Arial, sans-serif; background: #f4f4f4; margin: 0; display: flex; justify-content: center; align-items: center; height: 100vh; }
    .container { background: #fff; padding: 30px 20px; border-radius: 12px; box-shadow: 0 4px 10px rgba(0,0,0,0.1); text-align: center; }
    h2 { color: #333; margin-bottom: 20px; }
    .controls { display: flex; flex-direction: column; align-items: center; gap: 14px; }
    .row { display: flex; gap: 14px; }
    button {
      width: 90px; height: 90px; border: none; border-radius: 50%;
      background: #007bff; color: #fff; font-size: 2.5rem; font-weight: bold;
      box-shadow: 0 2px 6px rgba(0,0,0,0.08); cursor: pointer; transition: background 0.2s;
      outline: none;
      display: flex; align-items: center; justify-content: center;
    }
    button:active { background: #0056b3; }
    .horn { background: #ff9800; color: #fff; }
    .horn:active { background: #e68900; }
    .switch, .home {
      width: 100%; border-radius: 8px; background: #222; color: #fff; font-size: 1.1rem; margin-top: 18px; height: 48px;
    }
    .switch:active, .home:active { background: #444; }
    @media (max-width: 500px) {
      .container { padding: 15px 5px; }
      button { width: 60px; height: 60px; font-size: 1.5rem; }
      .switch, .home { height: 36px; font-size: 1rem; }
    }
    #statusbar { margin-bottom: 10px; }
    #feedback { margin-top: 18px; font-weight: bold; color: #555; }
  </style>
  <script>
    function updateStatusBar() {
      fetch('/status').then(r=>r.text()).then(html=>{document.getElementById('statusbar').innerHTML=html;});
    }
    function sendCmd(cmd) {
      fetch('/move?cmd='+cmd)
        .then(r=>r.text())
        .then(txt=>{
          document.getElementById('feedback').innerText = "Last Command: " + txt;
          updateStatusBar();
        });
    }
    let holdInterval;
    function startCmd(cmd) {
      sendCmd(cmd);
      holdInterval = setInterval(() => sendCmd(cmd), 300);
    }
    function stopCmd() {
      clearInterval(holdInterval);
      sendCmd('stop');
    }
    function hornOn() { sendCmd('horn_on'); }
    function hornOff() { sendCmd('horn_off'); }
    function gotoMode() { window.location.href = "/"; }
    window.onload = updateStatusBar;
    setInterval(updateStatusBar, 5000);
  </script>
</head>
<body>
  <div class="container">
    <div id="statusbar"></div>
    <h2>Wheelchair Car Control</h2>
    <div class="controls">
      <div class="row">
        <button ontouchstart="startCmd('forward')" ontouchend="stopCmd()" onmousedown="startCmd('forward')" onmouseup="stopCmd()" onmouseleave="stopCmd()">&#8593;</button>
      </div>
      <div class="row">
        <button ontouchstart="startCmd('left')" ontouchend="stopCmd()" onmousedown="startCmd('left')" onmouseup="stopCmd()" onmouseleave="stopCmd()">&#8592;</button>
        <button ontouchstart="startCmd('stop')" ontouchend="stopCmd()" onmousedown="startCmd('stop')" onmouseup="stopCmd()" onmouseleave="stopCmd()">&#9632;</button>
        <button ontouchstart="startCmd('right')" ontouchend="stopCmd()" onmousedown="startCmd('right')" onmouseup="stopCmd()" onmouseleave="stopCmd()">&#8594;</button>
      </div>
      <div class="row">
        <button ontouchstart="startCmd('backward')" ontouchend="stopCmd()" onmousedown="startCmd('backward')" onmouseup="stopCmd()" onmouseleave="stopCmd()">&#8595;</button>
        <button class="horn" ontouchstart="hornOn()" ontouchend="hornOff()" onmousedown="hornOn()" onmouseup="hornOff()" onmouseleave="hornOff()">🔊</button>
      </div>
    </div>
    <button class="switch" onclick="gotoMode()">Back to Mode Selection</button>
    <button class="home" onclick="gotoMode()">Home</button>
    <div id="feedback">Last Command: Stopped</div>
  </div>
</body>
</html>
)rawliteral";

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);

  // Pin setup
  pinMode(LM_IN1, OUTPUT); pinMode(LM_IN2, OUTPUT);
  pinMode(RM_IN3, OUTPUT); pinMode(RM_IN4, OUTPUT);
  pinMode(hallSensor1, INPUT); pinMode(hallSensor2, INPUT); pinMode(hallSensor3, INPUT);
  pinMode(xAxis_pin, INPUT); pinMode(yAxis_pin, INPUT);
  pinMode(button_pin, INPUT_PULLUP);
  pinMode(buzzer_pin, OUTPUT);
  pinMode(status_led, OUTPUT);

  digitalWrite(LM_IN1, LOW); digitalWrite(LM_IN2, LOW);
  digitalWrite(RM_IN3, LOW); digitalWrite(RM_IN4, LOW);
  digitalWrite(buzzer_pin, LOW);
  digitalWrite(status_led, LOW);

  // Allocate memory for movement sequence
  seq_Array = (Movement*)malloc(seqArraySize * sizeof(Movement));
  if (seq_Array == NULL) {
    Serial.println("Memory allocation failed!");
    while (1);
  }

  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(status_led, !digitalRead(status_led));
  }
  Serial.println("\nWiFi Connected!");
  Serial.println(WiFi.localIP());
  digitalWrite(status_led, HIGH);

  // mDNS
  if (MDNS.begin(mdnsName)) {
    Serial.println("mDNS responder started: http://" + String(mdnsName) + ".local/");
  } else {
    Serial.println("Error setting up mDNS responder!");
  }

  // Watchdog timer (10s timeout)
  esp_task_wdt_init(10, true);
  esp_task_wdt_add(NULL);

  // Web server endpoints
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", mode_webpage);
  });
  server.on("/car", HTTP_GET, []() {
    server.send(200, "text/html", car_webpage);
  });
  server.on("/status", HTTP_GET, []() {
    server.send(200, "text/html", getStatusBar());
  });
  server.on("/setMode", HTTP_GET, []() {
    if (server.hasArg("mode") && (millis() - lastModeChange > modeChangeCooldown)) {
      mode = server.arg("mode");
      lastModeChange = millis();
      Serial.println("Mode changed to: " + mode);
      server.send(200, "text/plain", mode);
    } else {
      server.send(200, "text/plain", mode);
    }
  });
  server.on("/move", HTTP_GET, handleMove);
  server.begin();
}

// =================== MAIN LOOP ===================
void loop() {
  server.handleClient();
  MDNS.update();
  maintainWiFi();
  updateBuzz();
  updateReplay();
  esp_task_wdt_reset(); // Reset watchdog

  // Status LED: solid if WiFi, blink if not
  digitalWrite(status_led, WiFi.status() == WL_CONNECTED ? HIGH : (millis() % 1000 < 500 ? HIGH : LOW));

  // --- Main control logic ---
  if (mode == "joystick") {
    handleButtonPress();
    char value = readJoystick();
    if (value != lastMovement) {
      if (isRecording) {
        int duration = millis() - moveStartTime;
        if (lastMovement != 'S') updateSeq(mapJoystickToMovement(lastMovement), duration);
        moveStartTime = millis();
      }
      lastMovement = value;
    }
    if (!replaying && !isReplaying && value != 0) {
      check_Inst(value);
    } else if (!replaying && !isReplaying) {
      go_Stop();
    }
  } else if (mode == "eye") {
    static bool eyeModeActivated = false;
    if (!eyeModeActivated) {
      Serial.println("EYE_MODE");
      eyeModeActivated = true;
    }
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command == "up") go_Forward();
      else if (command == "down") go_Backward();
      else if (command == "left" || command == "left close") go_Left();
      else if (command == "right" || command == "right close") go_Right();
      else if (command == "center" || command == "both close") go_Stop();
      else if (command == "system_off") { go_Stop(); eyeModeActivated = false; }
    }
  } else if (mode == "hall") {
    int s1 = digitalRead(hallSensor1);
    int s2 = digitalRead(hallSensor2);
    int s3 = digitalRead(hallSensor3);
    if (s1 == LOW) go_Left();
    else if (s2 == HIGH) go_Forward();
    else if (s3 == LOW) go_Right();
    else go_Stop();
  }
}

// =================== WIFI MANAGEMENT ===================
void maintainWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.reconnect();
  }
}

// =================== DEBOUNCED COMMAND HANDLER ===================
void handleMove() {
  unsigned long now = millis();
  if (now - lastCmdTime < debounceMs) {
    server.send(429, "text/plain", lastCommand);
    return;
  }
  lastCmdTime = now;
  String cmd = server.arg("cmd");
  if (cmd == "forward")      { go_Forward(); lastCommand = "Forward"; }
  else if (cmd == "backward"){ go_Backward(); lastCommand = "Backward"; }
  else if (cmd == "left")    { go_Left(); lastCommand = "Left"; }
  else if (cmd == "right")   { go_Right(); lastCommand = "Right"; }
  else if (cmd == "stop")    { go_Stop(); lastCommand = "Stopped"; }
  else if (cmd == "horn_on") { digitalWrite(buzzer_pin, HIGH); hornActive = true; lastCommand = "Horn On"; }
  else if (cmd == "horn_off"){ digitalWrite(buzzer_pin, LOW); hornActive = false; lastCommand = "Horn Off"; }
  else { lastCommand = "Unknown"; }
  server.send(200, "text/plain", lastCommand);
}

// =================== INPUT HANDLING ===================
char readJoystick() {
  int xAxis = analogRead(xAxis_pin);
  int yAxis = analogRead(yAxis_pin);
  // Calibrated thresholds (adjust as needed)
  if (xAxis > 3000 && abs(yAxis - 2048) < 500) return 'F'; // Forward
  if (yAxis > 3000 && abs(xAxis - 2048) < 500) return 'L'; // Left
  if (yAxis < 1000 && abs(xAxis - 2048) < 500) return 'R'; // Right
  return 'S'; // Stop
}

void handleButtonPress() {
  static bool lastButtonState = HIGH;
  bool buttonState = digitalRead(button_pin);
  if (buttonState == LOW && lastButtonState == HIGH) {
    buttonPressStart = millis();
  }
  if (buttonState == LOW && (millis() - buttonPressStart > 50)) { // Debounce
    buttonPressDuration = millis() - buttonPressStart;
    if (buttonPressDuration >= 2000 && buttonPressDuration < 4000 && !isRecording) {
      Serial.println("Path recording started...");
      isRecording = true;
      seq = 0;
      moveStartTime = millis();
      startBuzz(500);
    } else if (buttonPressDuration >= 4000 && buttonPressDuration < 6000 && isRecording) {
      Serial.println("Path replay initiated...");
      isRecording = false;
      rotate180Degrees();
      isReplaying = true;
      replayReverse = false;
      startReplay(false);
      startBuzz(1000);
    } else if (buttonPressDuration >= 6000 && isRecording) {
      Serial.println("Reverse replay initiated...");
      isRecording = false;
      isReversing = true;
      rotate180Degrees();
      isReplaying = true;
      replayReverse = true;
      startReplay(true);
      startBuzz(1500);
    }
  }
  if (buttonState == HIGH) {
    buttonPressStart = 0;
    buttonPressDuration = 0;
  }
  lastButtonState = buttonState;
}

// =================== MOTOR CONTROL ===================
void check_Inst(char value) {
  switch (value) {
    case 'F': go_Forward(); break;
    case 'L': go_Left(); break;
    case 'R': go_Right(); break;
    case 'S': go_Stop(); break;
  }
}

void go_Forward() {
  digitalWrite(LM_IN1, HIGH); digitalWrite(LM_IN2, LOW);
  digitalWrite(RM_IN3, HIGH); digitalWrite(RM_IN4, LOW);
}
void go_Backward() {
  digitalWrite(LM_IN1, LOW); digitalWrite(LM_IN2, HIGH);
  digitalWrite(RM_IN3, LOW); digitalWrite(RM_IN4, HIGH);
}
void go_Left() {
  digitalWrite(LM_IN1, LOW); digitalWrite(LM_IN2, HIGH);
  digitalWrite(RM_IN3, HIGH); digitalWrite(RM_IN4, LOW);
}
void go_Right() {
  digitalWrite(LM_IN1, HIGH); digitalWrite(LM_IN2, LOW);
  digitalWrite(RM_IN3, LOW); digitalWrite(RM_IN4, HIGH);
}
void go_Stop() {
  digitalWrite(LM_IN1, LOW); digitalWrite(LM_IN2, LOW);
  digitalWrite(RM_IN3, LOW); digitalWrite(RM_IN4, LOW);
}
void rotate180Degrees() {
  digitalWrite(LM_IN1, HIGH); digitalWrite(LM_IN2, LOW);
  digitalWrite(RM_IN3, LOW); digitalWrite(RM_IN4, HIGH);
  delay(600); // Adjust for your hardware
  go_Stop();
}

// =================== BUZZER (NON-BLOCKING) ===================
void startBuzz(int duration) {
  digitalWrite(buzzer_pin, HIGH);
  buzzStart = millis();
  buzzDuration = duration;
  buzzing = true;
}
void updateBuzz() {
  if (buzzing && millis() - buzzStart >= buzzDuration) {
    digitalWrite(buzzer_pin, LOW);
    buzzing = false;
  }
}

// =================== SEQUENCE MANAGEMENT ===================
void updateSeq(int movement, int duration) {
  if (seq >= seqArraySize) {
    int newSize = seqArraySize * 2;
    Movement* newArray = (Movement*)realloc(seq_Array, newSize * sizeof(Movement));
    if (newArray == NULL) {
      Serial.println("Memory reallocation failed!");
      while (1);
    }
    seq_Array = newArray;
    seqArraySize = newSize;
  }
  seq_Array[seq].type = movement;
  seq_Array[seq].duration = duration;
  seq++;
}
int mapJoystickToMovement(char joystickCommand) {
  switch (joystickCommand) {
    case 'F': return 1;
    case 'L': return 2;
    case 'R': return 3;
    case 'S': return 5;
    default: return 0;
  }
}
void clearStoredPath() {
  seq = 0;
  memset(seq_Array, 0, seqArraySize * sizeof(Movement));
}

// =================== REPLAY (NON-BLOCKING) ===================
void startReplay(bool reverse) {
  replaying = true;
  replayIndex = reverse ? seq - 1 : 0;
  replayMoveStart = millis();
  replayReverse = reverse;
}
void updateReplay() {
  if (!replaying || seq == 0) return;
  if (replayIndex < 0 || replayIndex >= seq) {
    go_Stop();
    replaying = false;
    isReplaying = false;
    isReversing = false;
    clearStoredPath();
    return;
  }
  Movement m = seq_Array[replayIndex];
  if (millis() - replayMoveStart == 0) {
    // Start movement
    switch (m.type) {
      case 1: go_Forward(); break;
      case 2: go_Left(); break;
      case 3: go_Right(); break;
      case 5: go_Stop(); break;
    }
  }
  if (millis() - replayMoveStart >= m.duration) {
    replayMoveStart = millis();
    replayIndex += replayReverse ? -1 : 1;
  }
}