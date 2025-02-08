# excavator-controller
Software for ESP32-based RC Excavator by ProfessorBoots

This repo holds the firmware for the ESP32-based contoller for the ProfessorBoots excavator. We have designed a new controller which supports the following improvements:

1. Integrated USB-C lithium battery charging and code download
2. PWM control of all motors
3. Battery level sense circuit
4. Audio capability and speaker
5. Internal OLED display for no particular reason

## ESP32 pins

Here is the IO map of the ESP32 pins:

| IO Pin   |  Function   |  Description  |
|----------|-------------|---------------|
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

Because there was plenty of space on the board, and because it's useful for debugging, I added a small OLED display. [Datasheet](https://www.lcsc.com/datasheet/lcsc_datasheet_2410121827_HS-HS13L03W2C01_C7465997.pdf). Its on the I2C bus and might be at address 0x3C. I will have to double check because the datasheet seems to have confused I2C with SPI.

 
