# Excavator Controller
Advanced ESP32-based RC Excavator Controller with integrated audio, lighting, and motor control systems.

## Hardware Features
- ESP32 microcontroller with USB-C interface
- PCA9635 16-channel PWM motor driver
- MAX98357 I2S audio amplifier
- WS2812 RGB beacon
- OLED display
- Integrated battery management

## Pin Mapping
| IO Pin | Function | Description |
|--------|----------|-------------|
| IO 2     |  BATT_STAT  | Low = battery trickle charing. Might not be useful. |
| IO 5     |  BEACON     | Data for WS2812 style RGB LED for beacon |
| IO 12    |  VBATT_ADC_EN | Set pin HIGH and wait 10mS to take ADC reading. Then set low. |
| IO 15    |  VBATT_ADC  | ADC input from battery, divided by 2.625  |
| IO 4     |  SERVO 3    | PPM output for Servo 3 |
| IO 16    |  SERVO 2    | PPM output for Servo 2 |
| IO 17    |  SERVO 1    | PPM output for Servo 1 |
| IO 14    |  I2S BCLK   | Bit clock for I2S Audio |
| IO 27    |  I2S LRCLK  | Word select (L/R select) for I2S audio. Only the LEFT channel is used |
| IO 26    | I2S DOUT | Data out for I2S Audio |
| IO 33    | I2C SDA  | I2C Data line |
| IO 32    | I2C SCL  | I2C Clock line |
| IO 35    | OE       | Active low Output Enable for PCA9635. Needs to be mapped elsewhere as 35 is input-only. |
| IO 21    | AUX LED  | Auxiliary LED, high = on |
| IO 19    | CAB LED  | Front cab LEDs, high = on |
| IO 18    | BOOM LED | Boom LEDs, high = on |
| IO 22    | 5V EN    | Enables the 5V regulator for servos, LEDs and audio |
| IO 23    | MOTORS EN | High = enable all motors, low = disable all motors |

## Motor Control System
The PCA9635 provides 16 PWM channels mapped as 8 dual-channel motor drivers.

### Motor Functions

// Set motor speed from -255 (full reverse) to 255 (full forward)
setBoom(int16_t speed);      // Control main boom
setDipper(int16_t speed);    // Control dipper arm
setBucket(int16_t speed);    // Control bucket
setThumb(int16_t speed);     // Control thumb grab
setRotator(int16_t speed);   // Control cab rotation
setLeftTrack(int16_t speed); // Control left track
setRightTrack(int16_t speed);// Control right track
setPusher(int16_t speed);    // Control auxiliary pusher

// Example usage:
setBoom(255);      // Full up
setBucket(-128);   // Half speed down
setRotator(0);     // Stop rotation


Serial 0 on the ESP32 is mapped to the USB-C port, with an auto-reset and bootloader circuit compatible with Arduino / PlatformIO.

## Motor drives

The motors are all connected to a PCA9635 16-channel PWM driver. Each motor has 2 inputs, and are used thus:

| Motor A | Motor B | Result   |
|----------|---------|------------|
| 0 | 0 | Motor off, free running |
| 0 | PWM | Motor clockwise, PWM = speed |
| PWM | 0 | Motor counter-clockwise, PWM = speed |
| 1  |  1 | Motor brake |
------------------------

[PCA9635 datasheet](https://www.nxp.com/docs/en/data-sheet/PCA9635.pdf)

The i2c address is set to 0x00 which is an all-call address. I will have to modify the boards to set at least one of these pins high.

The channels are mapped as follows:

| LEDx output  | Motor channel |
|--------------|---------------|
| 0  | 1A |
| 1  | 1B | 
| 2  | 2A |
| 3  | 2B |
| 4 | 3A |
| 5 | 3B |
| 6 | 4A |
| 7 | 4B |
| 8 | 5A |
| 9 | 5B |
| 10 | 6A |
| 11 | 6B |
| 12 | 7A |
| 13 | 7B |
| 14 | 8A |
| 15 | 8B |
----------

The motors are mapped as follows:

| Motor | Physical Function |
|-------|----------------------|
| 1     | Boom |
| 2     | Dipper |
| 3     | Bucket |
| 4     | Thumb |
| 5     | Rotator |
| 6     | Left Track | 
| 7     | Right Track |
| 8     | Pusher |



## Audio

There is a MAX983577 [datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/max98357a-max98357b.pdf) audio amplifier on the I2S bus. This will drive a 1W speaker.

WAV files can be stored on the ESP32's internal flash and played. I suggest we get the following:

* Engine start
* Engine idle
* Engine power up
* Engine running at power / hydraulic pump running
* Engine power down
* Engine stop
* Reversing beep
* Horn

## Display

Because there was plenty of space on the board, and because it's useful for debugging, I added a small OLED display. [Datasheet](https://www.lcsc.com/datasheet/lcsc_datasheet_2410121827_HS-HS13L03W2C01_C7465997.pdf). Its on the I2C bus and might be at address 0x3C. ~~ I will have to double check because the datasheet seems to have confused I2C with SPI.~~ The display will not be fitted as it is SPI and not I2C.

## Usage Examples

### Initializing the System

void setup() {
    setupMotors();
    setupAudio();
    setupLights();
    setupBeacon();
    setupBatteryMonitor();
}

void loop() {
    // Basic excavator operation sequence
    setBeacon(true);
    startEngine();
    delay(2000);
    
    setCabLight(255);
    setBoomLight(128);
    
    setBoom(128);      // Half speed up
    setRotator(64);    // Quarter speed rotate
    delay(1000);
    
    enginePowerUp();   // Increase power for work
    setBucket(-192);   // Dig operation
    setThumb(255);     // Grab
    
    // Continue with operation sequence...
}


### Controlling Motors

void setMotor(uint8_t motor, int16_t speed) {
    uint8_t channelA = motor * 2 - 2;
    uint8_t channelB = motor * 2 - 1;
    
    if (speed > 0) {
        setPWM(channelA, 0);
        setPWM(channelB, speed);
    } else if (speed < 0) {
        setPWM(channelA, -speed);
        setPWM(channelB, 0);
    } else {
        setPWM(channelA, 0);
        setPWM(channelB, 0);
    }
}

void setPWM(uint8_t channel, uint8_t value) {
    Wire.beginTransmission(PCA9635_ADDRESS);
    Wire.write(PCA9635_PWM0 + channel);
    Wire.write(value);
    Wire.endTransmission();
}


### Reading Battery Level

uint16_t readBatteryVoltage() {
    digitalWrite(IO_VBATT_ADC_EN, HIGH);
    delay(10);
    int adcValue = analogRead(IO_VBATT_ADC);
    digitalWrite(IO_VBATT_ADC_EN, LOW);
    
    return (uint16_t)((adcValue / 4095.0) * 3300 * 2.625);
}


### Playing Audio

void playAudio(const uint8_t* wavData, size_t wavSize) {
    size_t bytesWritten;
    i2s_write(I2S_NUM_0, wavData, wavSize, &bytesWritten, portMAX_DELAY);
}


### Updating Display

void updateDisplay(float batteryVoltage, int motorSpeed) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.print("Battery: ");
    display.print(batteryVoltage);
    display.println("V");
    display.print("Motor Speed: ");
    display.println(motorSpeed);
    display.display();
}


These examples demonstrate how to initialize the system, control motors, read battery levels, play audio, and update the display. For full functionality, you would combine these in your main loop and add additional logic for specific excavator operations.

# Warning Beacon
WS2812 RGB LED beacon with multiple modes.

## Beacon Functions
```
setBeacon(bool enabled);  // Enable/disable beacon
// Automatically switches between:
// - Normal: Orange flashing at 80ms
// - Low Battery: Red flashing at 500ms

// Example usage:
setBeacon(true);   // Enable beacon
setBeacon(false);  // Disable beacon
```
## Audio System
I2S-based audio playback system with DMA support and asynchronous operation.

## Audio Functions
```
startEngine();      // Start sequence: start.wav → idle.wav (loop)
enginePowerUp();    // Power up sequence: powerup.wav → power.wav (loop)
engineHydraulic();  // Hydraulic loop: hydraulic.wav (loop)
enginePowerDown();  // Power down sequence: powerdown.wav → idle.wav (loop)
stopEngine();       // Stop sequence: stop.wav

// Direct audio control
playWAVAsync(const char* filename, bool loop = false);
stopAudio();

// Example usage:
startEngine();  // Start the engine
delay(5000);    // Run for 5 seconds
enginePowerUp(); // Increase power
```

## Lighting System

PWM-controlled LED outputs for cab, boom, and auxiliary lighting.

## Light Functions

```
setCabLight(uint8_t brightness);    // 0-255 brightness
setBoomLight(uint8_t brightness);   // 0-255 brightness
setAuxLight(uint8_t brightness);    // 0-255 brightness
allLightsOn(uint8_t brightness);    // Set all lights
allLightsOff();                     // Turn off all lights

// Example usage:
setCabLight(255);    // Full brightness
setBoomLight(128);   // Half brightness
allLightsOn(64);     // All lights at quarter brightness
```

## Battery Monitoring
Automatic voltage monitoring with low battery warnings.

## Battery System
* Monitors every 10 seconds
* Low battery threshold: 3500mV
* Triggers audio warning and red beacon when low
* Voltage divider ratio: 2.625

```
uint16_t voltage = readBatteryVoltage();  // Returns millivolts
```

## Thread Safety
The system uses FreeRTOS tasks for:

* Audio playback (4KB stack)
* Battery monitoring (2KB stack)
* Main control loop

## Mutex protection for:

* Audio playback queue
* Battery state updates
* Beacon state changes

## Audio Files
## Required WAV files (44.1kHz, 16-bit, mono):

* start.wav: Engine startup sound
* idle.wav: Engine idle loop
* powerup.wav: Power increase sound
* power.wav: Full power loop
* hydraulic.wav: Hydraulic system loop
* powerdown.wav: Power decrease sound
* stop.wav: Engine shutdown sound
* low-batt.wav: Low battery warning

## Example Implementation

```
void setup() {
    setupMotors();
    setupAudio();
    setupLights();
    setupBeacon();
    setupBatteryMonitor();
}

void loop() {
    // Basic excavator operation sequence
    setBeacon(true);
    startEngine();
    delay(2000);
    
    setCabLight(255);
    setBoomLight(128);
    
    setBoom(128);      // Half speed up
    setRotator(64);    // Quarter speed rotate
    delay(1000);
    
    enginePowerUp();   // Increase power for work
    setBucket(-192);   // Dig operation
    setThumb(255);     // Grab
    
    // Continue with operation sequence...
}
```
## Build Requirements

* Platform: ESP32
* Lbraries:
  * PCA9635
  * Adafruit_NeoPixel
  * ESP32 Arduino Core
  * FreeRTOS
  * SPIFFS
