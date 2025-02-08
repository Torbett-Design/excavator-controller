#include <Arduino.h>
#include <PCA9635.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <driver/i2s.h>
#include <Adafruit_NeoPixel.h>
#include <mutex>
#include <queue>
#include <Ps3Controller.h>

#define PS3_CONTROLLER_MAC "01:02:03:04:05:06"

// PCA9635 instance
PCA9635 motorDriver(0x00);

// Light control definitions
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8  // 8-bit resolution (0-255)


// Orange construction beacon color
#define BEACON_COLOR beacon.Color(255, 140, 0)
#define BEACON_FLASH_INTERVAL 80  // Flash rate in ms


// PWM channels for lights
#define CAB_LIGHT_CHANNEL 0
#define BOOM_LIGHT_CHANNEL 1
#define AUX_LIGHT_CHANNEL 2

// I2S configuration
#define I2S_SAMPLE_RATE     44100
#define I2S_CHANNEL_NUM     1
#define I2S_BITS_PER_SAMPLE 16
#define DMA_BUF_COUNT       8
#define DMA_BUF_LEN         1024


// Audio task definitions
#define AUDIO_QUEUE_SIZE 10
#define AUDIO_TASK_STACK 4096

// Battery monitoring constants
#define BATTERY_CHECK_INTERVAL_MS 10000
#define BATTERY_LOW_THRESHOLD 3500
#define ADC_VOLTAGE_DIVIDER 2.625
#define BATTERY_RED_COLOR beacon.Color(255, 0, 0)
#define BATTERY_LOW_FLASH_INTERVAL 500


// Pin Definitions
#define BATT_STAT 2
#define BEACON 5
#define VBATT_ADC_EN 12
#define VBATT_ADC 15
#define SERVO_3 4
#define SERVO_2 16
#define SERVO_1 17
#define I2S_BCLK 14
#define I2S_LRCLK 27
#define I2S_DOUT 26
#define I2C_SDA 33
#define I2C_SCL 32
#define OE 35
#define AUX_LED 21
#define CAB_LED 19
#define BOOM_LED 18
#define V5_EN 22
#define MOTORS_EN 23

// NeoPixel beacon instance
#define BEACON_LED_COUNT 1
Adafruit_NeoPixel beacon(BEACON_LED_COUNT, BEACON, NEO_GRB + NEO_KHZ800);


// Audio paths
const char* SOUND_LOW_BATTERY = "/low-batt.wav";

// Thread safety
std::mutex audioMutex;
volatile bool lowBatteryState = false;


struct AudioCommand {
    const char* filename;
    bool loop;
};

QueueHandle_t audioQueue;
TaskHandle_t audioTaskHandle;
volatile bool audioStopRequest = false;

/* 
  Audio Task 
  This task handles audio playback from SPIFFS.
  @brief Plays audio files from SPIFFS using I2S.
*/
void audioTask(void* parameter) {
    AudioCommand cmd;
    
    while (true) {
        if (xQueueReceive(audioQueue, &cmd, portMAX_DELAY) == pdTRUE) {
            File file = SPIFFS.open(cmd.filename);
            if (!file) continue;

            file.seek(44); // Skip WAV header
            size_t bytesRead;
            uint8_t buffer[1024];
            
            do {
                bytesRead = file.read(buffer, sizeof(buffer));
                if (bytesRead > 0) {
                    size_t bytesWritten;
                    i2s_write(I2S_NUM_0, buffer, bytesRead, &bytesWritten, portMAX_DELAY);
                }
                
                if (cmd.loop && file.position() >= file.size()) {
                    file.seek(44);
                }
                
                // Check for stop request or new audio in queue
                if (audioStopRequest || uxQueueMessagesWaiting(audioQueue) > 0) {
                    break;
                }
                
            } while (bytesRead > 0);
            
            file.close();
            audioStopRequest = false;
        }
    }
}

/*
  Setup I2S
  @brief Configures I2S for audio playback.

*/
void setupI2S() {
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = I2S_SAMPLE_RATE,
      .bits_per_sample = (i2s_bits_per_sample_t)I2S_BITS_PER_SAMPLE,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = DMA_BUF_COUNT,
      .dma_buf_len = DMA_BUF_LEN,
      .use_apll = false,
      .tx_desc_auto_clear = true,
      .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_BCLK,
      .ws_io_num = I2S_LRCLK,
      .data_out_num = I2S_DOUT,
      .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}



// Audio file paths
const char* SOUND_START = "/start.wav";
const char* SOUND_IDLE = "/idle.wav";
const char* SOUND_POWERUP = "/powerup.wav";
const char* SOUND_POWER = "/power.wav";
const char* SOUND_HYDRAULIC = "/hydraulic.wav";
const char* SOUND_POWERDOWN = "/powerdown.wav";
const char* SOUND_STOP = "/stop.wav";

// Track currently playing looped sound
String currentLoopingSound = "";
bool isLooping = false;


// Motor channel definitions
enum MotorChannel {
  BOOM = 0,      // Motor 1 (channels 0,1)
  DIPPER = 2,    // Motor 2 (channels 2,3)
  BUCKET = 4,    // Motor 3 (channels 4,5)
  THUMB = 6,     // Motor 4 (channels 6,7)
  ROTATOR = 8,   // Motor 5 (channels 8,9)
  LEFT_TRACK = 10,  // Motor 6 (channels 10,11)
  RIGHT_TRACK = 12, // Motor 7 (channels 12,13)
  PUSHER = 14    // Motor 8 (channels 14,15)
};

bool beaconEnabled = false;
unsigned long lastBeaconUpdate = 0;
bool beaconState = false;

void setupBeacon() {
    beacon.begin();
    beacon.clear();
    beacon.show();
}

void setBeacon(bool enabled) {
    beaconEnabled = enabled;
    if (!enabled) {
        beacon.clear();
        beacon.show();
    }
}

void updateBeacon() {
    if (!beaconEnabled) return;
    
    unsigned long currentMillis = millis();
    if (currentMillis - lastBeaconUpdate >= BEACON_FLASH_INTERVAL) {
        beaconState = !beaconState;
        beacon.setPixelColor(0, beaconState ? BEACON_COLOR : 0);
        beacon.show();
        lastBeaconUpdate = currentMillis;
    }
}

void setMotor(MotorChannel motor, int16_t speed) {
  uint8_t channelA = motor;
  uint8_t channelB = motor + 1;
  
  // Constrain speed to valid range
  speed = constrain(speed, -255, 255);
  
  if (speed == 0) {
      // Motor off, free running
      motorDriver.write1(channelA, 0);
      motorDriver.write1(channelB, 0);
  }
  else if (speed > 0) {
      // Clockwise rotation
      motorDriver.write1(channelA, 0);
      motorDriver.write1(channelB, speed);
  }
  else {
      // Counter-clockwise rotation
      motorDriver.write1(channelA, -speed);
      motorDriver.write1(channelB, 0);
  }
}

// Convenience functions for each motor
void setBoom(int16_t speed) {
  setMotor(BOOM, speed);
}

void setDipper(int16_t speed) {
  setMotor(DIPPER, speed);
}

void setBucket(int16_t speed) {
  setMotor(BUCKET, speed);
}

void setThumb(int16_t speed) {
  setMotor(THUMB, speed);
}

void setRotator(int16_t speed) {
  setMotor(ROTATOR, speed);
}

void setLeftTrack(int16_t speed) {
  setMotor(LEFT_TRACK, speed);
}

void setRightTrack(int16_t speed) {
  setMotor(RIGHT_TRACK, speed);
}

void setPusher(int16_t speed) {
  setMotor(PUSHER, speed);
}


void setupAudioTask() {
  audioQueue = xQueueCreate(AUDIO_QUEUE_SIZE, sizeof(AudioCommand));
  
  xTaskCreate(
      audioTask,
      "AudioPlayer",
      AUDIO_TASK_STACK,
      NULL,
      1,
      &audioTaskHandle
  );
}

void playWAVAsync(const char* filename, bool loop = false) {
  AudioCommand cmd = {filename, loop};
  xQueueSend(audioQueue, &cmd, portMAX_DELAY);
}

void stopAudio() {
  audioStopRequest = true;
  while (!uxQueueMessagesWaiting(audioQueue)) {
      xQueueReset(audioQueue);
  }
}

// Update the low battery warning to use async playback
void playLowBatteryWarning() {
  std::lock_guard<std::mutex> lock(audioMutex);
  playWAVAsync(SOUND_LOW_BATTERY);
}

// Update engine sound functions
void startEngine() {
  playWAVAsync(SOUND_START);
  playWAVAsync(SOUND_IDLE, true);
}

void enginePowerUp() {
  playWAVAsync(SOUND_POWERUP);
  playWAVAsync(SOUND_POWER, true);
}

void engineHydraulic() {
  playWAVAsync(SOUND_HYDRAULIC, true);
}

void enginePowerDown() {
  playWAVAsync(SOUND_POWERDOWN);
  playWAVAsync(SOUND_IDLE, true);
}

void stopEngine() {
  playWAVAsync(SOUND_STOP);
}

void setupAudio() {
SPIFFS.begin(true);
setupI2S();
}


void setupLights() {
  // Configure LED PWM channels
  ledcSetup(CAB_LIGHT_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(BOOM_LIGHT_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(AUX_LIGHT_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);

  // Attach PWM channels to GPIO pins
  ledcAttachPin(CAB_LED, CAB_LIGHT_CHANNEL);
  ledcAttachPin(BOOM_LED, BOOM_LIGHT_CHANNEL);
  ledcAttachPin(AUX_LED, AUX_LIGHT_CHANNEL);
}

void setCabLight(uint8_t brightness) {
  ledcWrite(CAB_LIGHT_CHANNEL, brightness);
}

void setBoomLight(uint8_t brightness) {
  ledcWrite(BOOM_LIGHT_CHANNEL, brightness);
}

void setAuxLight(uint8_t brightness) {
ledcWrite(AUX_LIGHT_CHANNEL, brightness);
}

// Optional: Convenience functions for common operations
void allLightsOff() {
setCabLight(0);
setBoomLight(0);
setAuxLight(0);
}

void allLightsOn(uint8_t brightness = 255) {
setCabLight(brightness);
setBoomLight(brightness);
setAuxLight(brightness);
}


uint16_t readBatteryVoltage() {
  digitalWrite(VBATT_ADC_EN, HIGH);
  delay(10);
  uint16_t adcValue = analogRead(VBATT_ADC);
  digitalWrite(VBATT_ADC_EN, LOW);
    
  // Convert ADC reading to millivolts considering voltage divider
  return (uint16_t)(adcValue * (3300.0 / 4095.0) * ADC_VOLTAGE_DIVIDER);
}

void updateBeaconState() {
  unsigned long currentMillis = millis();
    
  if (!beaconEnabled) return;
    
  if (lowBatteryState) {
      // Slow red flashing for low battery
      if (currentMillis - lastBeaconUpdate >= BATTERY_LOW_FLASH_INTERVAL) {
          beaconState = !beaconState;
          beacon.setPixelColor(0, beaconState ? BATTERY_RED_COLOR : 0);
          beacon.show();
          lastBeaconUpdate = currentMillis;
      }
  } else {
      // Normal orange flashing
      if (currentMillis - lastBeaconUpdate >= BEACON_FLASH_INTERVAL) {
          beaconState = !beaconState;
          beacon.setPixelColor(0, beaconState ? BEACON_COLOR : 0);
          beacon.show();
          lastBeaconUpdate = currentMillis;
      }
  }
}


void batteryMonitorTask(void * parameter) {
  bool wasLowBattery = false;
    
  while(true) {
      uint16_t batteryVoltage = readBatteryVoltage();
      bool isLowBattery = batteryVoltage < BATTERY_LOW_THRESHOLD;
        
      if (isLowBattery && !wasLowBattery) {
          lowBatteryState = true;
          playLowBatteryWarning();
      }
        
      wasLowBattery = isLowBattery;
      lowBatteryState = isLowBattery;
        
      vTaskDelay(pdMS_TO_TICKS(BATTERY_CHECK_INTERVAL_MS));
  }
}

void setupBatteryMonitor() {
  pinMode(VBATT_ADC_EN, OUTPUT);
  pinMode(VBATT_ADC, INPUT);
  analogReadResolution(12);
    
  xTaskCreate(
      batteryMonitorTask,
      "BatteryMonitor",
      2048,
      NULL,
      1,
      NULL
  );
}

void setupPins() {
pinMode(BATT_STAT, INPUT);
pinMode(BEACON, OUTPUT);
pinMode(VBATT_ADC_EN, OUTPUT);
pinMode(VBATT_ADC, INPUT);
pinMode(SERVO_3, OUTPUT);
pinMode(SERVO_2, OUTPUT);
pinMode(SERVO_1, OUTPUT);
pinMode(AUX_LED, OUTPUT);
pinMode(CAB_LED, OUTPUT);
pinMode(BOOM_LED, OUTPUT);
pinMode(V5_EN, OUTPUT);
pinMode(MOTORS_EN, OUTPUT);

// Enable 5V and motors by default
digitalWrite(V5_EN, HIGH);
digitalWrite(MOTORS_EN, HIGH);
}

void setupMotorDriver() {
Wire.begin(I2C_SDA, I2C_SCL);
    
// Initialize PCA9635 with all-call address 0x00
motorDriver.begin(0x00);
    
// Set all channels to normal mode
motorDriver.setMode1(0);
motorDriver.setMode2(0x04);  // Enable output change on ACK

// Initialize all motor channels to OFF state
for (int i = 0; i < 16; i++) {
    motorDriver.write1(i, 0);
}
}

void ps3_notify()
{
  // add routines to get data from the controller
}

void setupController()
{
  Ps3.attach(ps3_notify);
  Ps3.begin(PS3_CONTROLLER_MAC);
}


void setup() {
    Serial.begin(115200);
    setupPins();
    setupMotorDriver();
    setupAudio();
    setupLights();
    setupBeacon();
    setupBatteryMonitor();
    setupAudioTask();
}
void loop() {
    // Main loop code will go here
    updateBeaconState();
}


