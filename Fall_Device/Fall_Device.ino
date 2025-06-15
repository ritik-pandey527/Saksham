#include <Fall_Detection_only_accelaration_inferencing.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <WiFi.h>
#include <HTTPClient.h>

// ------------------- CONFIGURATION -------------------
#define MAX_ACCEPTED_RANGE 156.91f
#define RUN_INFERENCE_PERIOD_MS 200
#define BUZZER_PIN D8
#define LED_PIN 2  // Onboard LED for status
#define BUZZER_ON_TIME_MS 5000
#define FALL_WINDOW_SIZE 5
#define FALL_THRESHOLD 3  // Number of positive detections in window to trigger alert
#define SMS_RETRY_COUNT 3
#define SMS_RETRY_DELAY_MS 5000
const unsigned long alert_interval = 30000;  // 30 seconds between alerts

// WiFi & Twilio Credentials (move to secure storage in production)
const char* ssid = "your_ssid";
const char* password = "your_password";
const char* twilio_account_sid = "your_account_sid";
const char* twilio_auth_token = "your_auth_token";
const char* twilio_number = "+1234567890";
const char* recipient_number = "+911234567890";

// ------------------- GLOBALS -------------------
float x_value = 0, y_value = 0, z_value = 0;
static bool debug_nn = false;
static float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
static float inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
SemaphoreHandle_t bufferMutex;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
TaskHandle_t inferenceTaskHandle;
TaskHandle_t smsTaskHandle = NULL;

unsigned long last_fall_alert_time = 0;
unsigned long last_fall_detected_time = 0;
bool buzzer_on = false;

// Sliding window for fall detection
bool fall_window[FALL_WINDOW_SIZE] = {false};
int fall_window_index = 0;

// ------------------- FUNCTION DECLARATIONS -------------------
void run_inference_background(void *pvParameters);
void send_sms_task(void *pvParameters);
void reconnectWiFi();
void update_fall_window(bool detected);
bool is_fall_triggered();
void indicate_status(bool wifi_ok, bool fall_detected);

// ------------------- SETUP -------------------
void setup() {
  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(500));

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  Serial.println("Starting Edge Impulse Inferencing using Transformer model");

  if (!accel.begin()) {
    Serial.println("Failed to initialize ADXL345! Rebooting...");
    ESP.restart();
  } else {
    Serial.println("ADXL345 initialized");
  }

  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_100_HZ);

  if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
    Serial.println("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be 3");
    return;
  }

  // Connect to WiFi
  reconnectWiFi();

  // Create mutex for buffer protection
  bufferMutex = xSemaphoreCreateMutex();

  // Start inference task
  xTaskCreate(run_inference_background, "InferenceTask", 4096, NULL, 1, &inferenceTaskHandle);
}

// ------------------- WIFI RECONNECT -------------------
void reconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink LED while connecting
  }
  Serial.println("Connected!");
  digitalWrite(LED_PIN, HIGH); // Solid LED when connected
}

// ------------------- FALL WINDOW LOGIC -------------------
void update_fall_window(bool detected) {
  fall_window[fall_window_index] = detected;
  fall_window_index = (fall_window_index + 1) % FALL_WINDOW_SIZE;
}

bool is_fall_triggered() {
  int count = 0;
  for (int i = 0; i < FALL_WINDOW_SIZE; i++) {
    if (fall_window[i]) count++;
  }
  return count >= FALL_THRESHOLD;
}

// ------------------- STATUS INDICATOR -------------------
void indicate_status(bool wifi_ok, bool fall_detected) {
  if (!wifi_ok) {
    digitalWrite(LED_PIN, millis() % 1000 < 500 ? HIGH : LOW); // Blink fast if WiFi lost
  } else if (fall_detected) {
    digitalWrite(LED_PIN, millis() % 1000 < 100 ? HIGH : LOW); // Blink short if fall
  } else {
    digitalWrite(LED_PIN, HIGH); // Solid if normal
  }
}

// ------------------- INFERENCE TASK -------------------
void run_inference_background(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS((EI_CLASSIFIER_INTERVAL_MS * EI_CLASSIFIER_RAW_SAMPLE_COUNT) + 100));

  while (1) {
    // Buffer protection
    xSemaphoreTake(bufferMutex, portMAX_DELAY);
    memcpy(inference_buffer, buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE * sizeof(float));
    xSemaphoreGive(bufferMutex);

    signal_t signal;
    int err = numpy::signal_from_buffer(inference_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
      Serial.printf("Failed to create signal (%d)\n", err);
      vTaskDelay(pdMS_TO_TICKS(RUN_INFERENCE_PERIOD_MS));
      continue;
    }

    ei_impulse_result_t result = {0};
    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
      Serial.printf("Classifier failed (%d)\n", err);
      vTaskDelay(pdMS_TO_TICKS(RUN_INFERENCE_PERIOD_MS));
      continue;
    }

    Serial.printf("ADXL345 Values: X=%.2f, Y=%.2f, Z=%.2f\n", x_value, y_value, z_value);
    Serial.printf("Predictions: Fall=%.5f, No Fall=%.5f\n",
                  result.classification[1].value, result.classification[0].value);

    bool fall_detected = result.classification[1].value > 0.7f;
    update_fall_window(fall_detected);

    if (is_fall_triggered()) {
      if (!buzzer_on) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzer_on = true;
        last_fall_detected_time = millis();
        Serial.println("Fall detected! Buzzer Started");
        indicate_status(true, true);

        // Send SMS in background task
        if (millis() - last_fall_alert_time > alert_interval) {
          if (smsTaskHandle == NULL) {
            xTaskCreate(send_sms_task, "SendSMSTask", 4096, NULL, 1, &smsTaskHandle);
          }
          last_fall_alert_time = millis();
        }
      }
    } else {
      if (buzzer_on && millis() - last_fall_detected_time >= BUZZER_ON_TIME_MS) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzer_on = false;
        Serial.println("Buzzer Stopped");
        indicate_status(true, false);
      }
    }

    // WiFi status indicator
    indicate_status(WiFi.status() == WL_CONNECTED, is_fall_triggered());

    // WiFi reconnect if needed
    if (WiFi.status() != WL_CONNECTED) {
      reconnectWiFi();
    }

    vTaskDelay(pdMS_TO_TICKS(RUN_INFERENCE_PERIOD_MS));
  }
}

// ------------------- SMS TASK WITH RETRY -------------------
void send_sms_task(void *pvParameters) {
  int retries = 0;
  bool sent = false;
  while (retries < SMS_RETRY_COUNT && !sent) {
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      String url = "https://api.twilio.com/2010-04-01/Accounts/" + String(twilio_account_sid) + "/Messages.json";
      String postData = "To=" + String(recipient_number) +
                        "&From=" + String(twilio_number) +
                        "&Body=Fall detected! Please check on the person immediately.";

      http.begin(url);
      http.setAuthorization(twilio_account_sid, twilio_auth_token);
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");

      int httpResponseCode = http.POST(postData);
      if (httpResponseCode > 0) {
        Serial.println("SMS Alert Sent!");
        sent = true;
      } else {
        Serial.printf("Error sending SMS: %d\n", httpResponseCode);
      }
      http.end();
    } else {
      Serial.println("WiFi not connected, SMS not sent!");
      reconnectWiFi();
    }
    if (!sent) {
      retries++;
      vTaskDelay(pdMS_TO_TICKS(SMS_RETRY_DELAY_MS));
    }
  }
  smsTaskHandle = NULL;
  vTaskDelete(NULL);
}

// ------------------- MAIN LOOP -------------------
void loop() {
  uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

  // Buffer protection
  xSemaphoreTake(bufferMutex, portMAX_DELAY);
  numpy::roll(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, -3);

  sensors_event_t event;
  accel.getEvent(&event);
  x_value = event.acceleration.x;
  y_value = event.acceleration.y;
  z_value = event.acceleration.z;

  buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3] = x_value;
  buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 2] = y_value;
  buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 1] = z_value;

  for (int i = 0; i < 3; i++) {
    if (fabs(buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3 + i]) > MAX_ACCEPTED_RANGE) {
      buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3 + i] =
          (buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3 + i] > 0 ? 1.0 : -1.0) * MAX_ACCEPTED_RANGE;
    }
  }
  xSemaphoreGive(bufferMutex);

  uint64_t current_time = micros();
  uint64_t time_to_wait = (current_time < next_tick) ? (next_tick - current_time) : (UINT64_MAX - current_time) + next_tick;
  vTaskDelay(pdMS_TO_TICKS(time_to_wait / 1000));
}