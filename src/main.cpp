#include <Arduino.h>
#include <Ps3Controller.h>
#include "audio.h"
#include "beacon.h"
#include "lights.h"
#include "motors.h"
#include "pins.h"

// #define PS3_CONTROLLER_MAC "01:02:03:04:05:06"

enum class State
{
    OFF,
    IDLE,
    POWER_UP,
    POWER,
    HYDRAULIC,
    POWER_DOWN,
    MOVING,
    REVERSE
};

State state = State::OFF;

static bool isMoving = false;
static bool startButtonPressed = false;
static unsigned long startButtonTime = 0;
bool tracksWereMoving = false;

static unsigned long lastActivityTime = 0;
const unsigned long IDLE_TIMEOUT = 3000; // 3 seconds

unsigned long lastDebugPrintTime = 0;
const unsigned long DEBUG_PRINT_INTERVAL = 1000; // 1 second

void ps3_notify()
{
    if (state != State::OFF)
    {
        // Forward movement using L2/R2 analog values (0-255)
        if (Ps3.data.analog.button.l2)
        {
            Motors::setLeftTrack(Ps3.data.analog.button.l2);
        }
        else if (Ps3.data.analog.button.l1)
        {
            Motors::setLeftTrack(-Ps3.data.analog.button.l1);
        }
        else
        {
            Motors::setLeftTrack(0);
        }

        if (Ps3.data.analog.button.r2)
        {
            Motors::setRightTrack(Ps3.data.analog.button.r2);
        }
        else if (Ps3.data.analog.button.r1)
        {
            Motors::setRightTrack(-Ps3.data.analog.button.r1);
        }
        else
        {
            Motors::setRightTrack(0);
        }
        
        // Use left analog stick for excavator rotation
        int8_t rotationValue = Ps3.data.analog.stick.lx;
        // Apply deadzone to prevent small unintended movements
        if (abs(rotationValue) < 15) {
            rotationValue = 0;
        }
        // Map from -128~127 range to -255~255 range
        int16_t mappedRotation = map(rotationValue, -128, 127, -255, 255);
        Motors::setRotator(mappedRotation);
    }

    // Handle start button state
    if (Ps3.data.button.start)
    {
        if (startButtonTime == 0)
        {
            startButtonTime = millis();
        }
        startButtonPressed = true;
    }
    else
    {
        startButtonPressed = false;
        startButtonTime = 0;
    }
}
void setupController()
{
    Ps3.attach(ps3_notify);
    Ps3.begin();
    String mac = Ps3.getAddress();
    Serial.println("MAC address: " + mac);
}

void setupPins()
{
    pinMode(Pins::BATT_STAT, INPUT);
    pinMode(Pins::VBATT_ADC_EN, OUTPUT);
    pinMode(Pins::VBATT_ADC, INPUT);
    pinMode(Pins::SERVO_3, OUTPUT);
    pinMode(Pins::SERVO_2, OUTPUT);
    pinMode(Pins::SERVO_1, OUTPUT);
    pinMode(Pins::V5_EN, OUTPUT);

    // Enable 5V by default
    digitalWrite(Pins::V5_EN, HIGH);
}

void handleShutdown()
{
    Audio::stopEngine();

    Motors::setBoom(0);
    Motors::setDipper(0);
    Motors::setBucket(0);
    Motors::setThumb(0);
    Motors::setRotator(0);
    Motors::setLeftTrack(0);
    Motors::setRightTrack(0);
    Motors::setPusher(0);

    Lights::setBoomLight(false);
    vTaskDelay(100);
    Lights::setCabLight(false);
    vTaskDelay(100);


    Beacon::setEnabled(false);

}

float getBatteryVoltage()
{
    digitalWrite(Pins::VBATT_ADC_EN, HIGH);
    vTaskDelay(10);
    int adcValue = analogRead(Pins::VBATT_ADC);
    digitalWrite(Pins::VBATT_ADC_EN, LOW);

    return (float)((adcValue / 4095.0) * 3300 * 2.625) / 1000.0; // Return in volts
}

void setup()
{
    Serial.begin(115200);
    setupPins();
    Motors::setup();
    Audio::setup();
    Lights::setup();
    Beacon::setup();
    setupController();
}

void loop()
{


    bool hydraulicActive = Motors::getBoom() != 0 || Motors::getDipper() != 0 ||
    Motors::getBucket() != 0 || Motors::getThumb() != 0 ||
    Motors::getRotator() != 0;
    bool tracksMoving = Motors::areTracksMoving();

    // State machine handling
    switch (state)
    {
    case State::OFF:
        if (startButtonPressed && (millis() - startButtonTime >= 100))
        {
            Audio::startEngine();
            Beacon::setEnabled(true);
            state = State::IDLE;
        }
        break;

    case State::IDLE:

    // Start appropriate sounds when activity begins
        if ((tracksMoving || hydraulicActive))
        {
            //Audio::enginePowerUp();
            state = State::MOVING;
        }

        if (startButtonPressed && (millis() - startButtonTime >= 2000))
        {
            Serial.println("Shutting Down");
            handleShutdown();
            state = State::OFF;
        }
        break;


    case State::MOVING:

        // Manage ongoing sounds based on activity
        if (tracksMoving)
        {
            //Audio::tracks();
        }
        else
        {
            //Audio::stopTrack(Audio::SOUND_TRACK);
        }

        if (hydraulicActive)
        {
            //Audio::engineHydraulic();
        }
        else
        {
            Audio::stopTrack(Audio::SOUND_HYDRAULIC);
        }

        // Return to idle after timeout
        if (!tracksMoving && !hydraulicActive)
        {
            if (lastActivityTime == 0)
            {
                lastActivityTime = millis();
            }
            else if (millis() - lastActivityTime >= IDLE_TIMEOUT)
            {
                //Audio::enginePowerDown();
                state = State::IDLE;
                lastActivityTime = 0;
            }
        }
        else
        {
            lastActivityTime = 0;
        }
        break;
    }
    // Rest of control loop code

        // Debug printing once per second
        unsigned long currentMillis = millis();
        if (currentMillis - lastDebugPrintTime >= DEBUG_PRINT_INTERVAL)
        {
            lastDebugPrintTime = currentMillis;

            // State information
            Serial.println("==== EXCAVATOR DEBUG INFO ====");

            // State
            Serial.print("System State: ");
            switch (state)
            {
            case State::OFF:
                Serial.println("OFF");
                break;
            case State::IDLE:
                Serial.println("IDLE");
                break;
            case State::POWER_UP:
                Serial.println("POWER_UP");
                break;
            case State::POWER:
                Serial.println("POWER");
                break;
            case State::HYDRAULIC:
                Serial.println("HYDRAULIC");
                break;
            case State::POWER_DOWN:
                Serial.println("POWER_DOWN");
                break;
            case State::MOVING:
                Serial.println("MOVING");
                break;
            case State::REVERSE:
                Serial.println("REVERSE");
                break;
            default:
                Serial.println("UNKNOWN");
            }

            // Battery
            float batteryVoltage = getBatteryVoltage();
            Serial.print("Battery: ");
            Serial.print(batteryVoltage, 2);
            Serial.println("V");

            // Charger status
            bool chargerActive = !digitalRead(Pins::BATT_STAT); // Low = charging
            Serial.print("Charger: ");
            Serial.println(chargerActive ? "CHARGING" : "NOT CHARGING");

            // Motor status
            Serial.print("Motors: Boom=");
            Serial.print(Motors::getBoom());
            Serial.print(" Dipper=");
            Serial.print(Motors::getDipper());
            Serial.print(" Bucket=");
            Serial.print(Motors::getBucket());
            Serial.print(" Thumb=");
            Serial.print(Motors::getThumb());
            Serial.print(" Rot=");
            Serial.println(Motors::getRotator());
        
            // Add a dedicated rotation status line
            Serial.print("Rotation: ");
            int16_t rotationValue = Motors::getRotator();
            Serial.print(rotationValue);
            if (rotationValue > 50) {
                Serial.println(" (Rotating RIGHT)");
            } else if (rotationValue < -50) {
                Serial.println(" (Rotating LEFT)");
            } else {
                Serial.println(" (Stopped)");
            }

            // Tracks
            Serial.print("Tracks: L=");
            Serial.print(Motors::getLeftTrack());
            Serial.print(" R=");
            Serial.print(Motors::getRightTrack());
            Serial.print(" Moving=");
            Serial.println(Motors::areTracksMoving() ? "YES" : "NO");

            // Beacon & Lights
            Serial.print("Beacon: ");
            Serial.print(Beacon::isEnabled() ? "ON" : "OFF");
            Serial.print(" Cab Light: ");
            Serial.print(Lights::isCabLightOn() ? "ON" : "OFF");
            Serial.print(" Boom Light: ");
            Serial.println(Lights::isBoomLightOn() ? "ON" : "OFF");

            // Uptime
            Serial.print("Uptime: ");
            Serial.print(currentMillis / 1000);
            Serial.println(" seconds");

            Serial.println("=============================");
        }
    vTaskDelay(10);
}
