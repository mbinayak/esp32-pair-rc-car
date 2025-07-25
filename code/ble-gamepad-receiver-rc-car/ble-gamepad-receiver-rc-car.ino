#include <Bluepad32.h>
#include <esp32-hal-ledc.h>

#define leftMotorPWMPin 14               // Left motor speed (PWM) pin
#define rightMotorPWMPin 32              // right motor speed (PWM) pin
#define leftMotorClockwiseDirPin 27      // Motor A direction pin 1
#define leftMotorAntiClockwiseDirPin 26  // Motor A direction pin 2
#define rightMotorClockwiseDirPin 25     // Motor B direction pin 1
#define rightMotorAntiClockwiseDirPin 33 // Motor B direction pin 2

// PWM ledc setup for older version 2.*
#define LEFT_MOTOR_CHANNEL 0
#define RIGHT_MOTOR_CHANNEL 1
#define PWM_FREQ 1000 // 1kHz
#define PWM_RES 8     // 8-bit (0–255)

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl)
{
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    {
        if (myControllers[i] == nullptr)
        {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                          properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot)
    {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl)
{
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    {
        if (myControllers[i] == ctl)
        {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController)
    {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl)
{
    // Serial.printf(
    //     "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    //     "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    //     ctl->index(),        // Controller Index
    //     ctl->dpad(),         // D-pad
    //     ctl->buttons(),      // bitmask of pressed buttons
    //     ctl->axisX(),        // (-511 - 512) left X Axis
    //     ctl->axisY(),        // (-511 - 512) left Y axis
    //     ctl->axisRX(),       // (-511 - 512) right X axis
    //     ctl->axisRY(),       // (-511 - 512) right Y axis
    //     ctl->brake(),        // (0 - 1023): brake button
    //     ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
    //     ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    //     ctl->gyroX(),        // Gyro X
    //     ctl->gyroY(),        // Gyro Y
    //     ctl->gyroZ(),        // Gyro Z
    //     ctl->accelX(),       // Accelerometer X
    //     ctl->accelY(),       // Accelerometer Y
    //     ctl->accelZ()        // Accelerometer Z
    // );

    Serial.printf(
        "idx=%d, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d\n",
        ctl->index(),   // Controller Index
        ctl->axisX(),   // (-511 - 512) left X Axis
        ctl->axisY(),   // (-511 - 512) left Y axis
        ctl->axisRX(),  // (-511 - 512) right X axis
        ctl->axisRY(),  // (-511 - 512) right Y axis
        ctl->brake(),   // (0 - 1023): brake button
        ctl->throttle() // (0 - 1023): throttle (AKA gas) button
    );
}

void processGamepad(ControllerPtr ctl)
{
    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    dumpGamepad(ctl);
    driveMotorsFromGamepad(ctl);
}

void processControllers()
{
    for (auto myController : myControllers)
    {
        if (myController && myController->isConnected() && myController->hasData())
        {
            if (myController->isGamepad())
            {
                processGamepad(myController);
            }
            else
            {
                Serial.println("Unsupported controller");
            }
        }
    }
}

void driveMotorsFromGamepad(ControllerPtr ctl)
{
    int throttle = ctl->throttle(); // 0–1023
    int brake = ctl->brake();       // 0–1023
    int16_t axisX = ctl->axisX();   // -511 to 512 (turn left by reducing left motor)
    int16_t axisRX = ctl->axisRX(); // -511 to 512 (turn right by reducing right motor)

    // Signed drive speed
    int drive = throttle - brake;
    // if (drive == 0) {
    //     setMotorV2(LEFT_MOTOR_CHANNEL, leftMotorClockwiseDirPin, leftMotorAntiClockwiseDirPin, 0);
    //     setMotorV2(RIGHT_MOTOR_CHANNEL, rightMotorClockwiseDirPin, rightMotorAntiClockwiseDirPin, 0);
    //     Serial.println("Drive = 0 → Full Stop");
    //     return;
    // }

    int baseSpeed = map(drive, -1023, 1023, -255, 255);

    // Apply turning effects
    int leftTurnOffset = map(axisX, -511, 508, -255, 255);
    int rightTurnOffset = map(axisRX, -511, 508, -255, 255);

    int leftSpeed = constrain(baseSpeed - leftTurnOffset, -255, 255);
    int rightSpeed = constrain(baseSpeed - rightTurnOffset, -255, 255);

    // Send to motors
    // setMotorV3(leftMotorPWMPin, leftMotorClockwiseDirPin, leftMotorAntiClockwiseDirPin, leftSpeed);
    // setMotorV3(rightMotorPWMPin, rightMotorClockwiseDirPin, rightMotorAntiClockwiseDirPin, rightSpeed);
    setMotorV2(LEFT_MOTOR_CHANNEL, leftMotorClockwiseDirPin, leftMotorAntiClockwiseDirPin, leftSpeed);
    setMotorV2(RIGHT_MOTOR_CHANNEL, rightMotorClockwiseDirPin, rightMotorAntiClockwiseDirPin, rightSpeed);

    Serial.printf("Drive: %d | L: %d | R: %d | LO: %d | RO: %d\n", baseSpeed, leftSpeed, rightSpeed, leftTurnOffset, rightTurnOffset);
}

void setMotorV2(int channel, int clockwiseDirPin, int antiClockwiseDirPin, int speed)
{
    if (speed > 0)
    {
        digitalWrite(clockwiseDirPin, HIGH);
        digitalWrite(antiClockwiseDirPin, LOW);
        ledcWrite(channel, speed);
    }
    else if (speed < 0)
    {
        digitalWrite(clockwiseDirPin, LOW);
        digitalWrite(antiClockwiseDirPin, HIGH);
        ledcWrite(channel, -speed);
    }
    else
    {
        digitalWrite(clockwiseDirPin, LOW);
        digitalWrite(antiClockwiseDirPin, LOW);
        ledcWrite(channel, 0);
    }
}

void setMotorV3(int pwmPin, int clockwiseDirPin, int antiClockwiseDirPin, int speed)
{
    if (speed > 0)
    {
        // move forward
        digitalWrite(clockwiseDirPin, HIGH);
        digitalWrite(antiClockwiseDirPin, LOW);
        ledcWrite(pwmPin, speed); // instead of analogWrite
    }
    else if (speed < 0)
    {
        // move backwards
        digitalWrite(clockwiseDirPin, LOW);
        digitalWrite(antiClockwiseDirPin, HIGH);
        ledcWrite(pwmPin, -speed); // instead of analogWrite
    }
    else
    {
        // Stop movement
        digitalWrite(clockwiseDirPin, LOW);
        digitalWrite(antiClockwiseDirPin, LOW);
        ledcWrite(pwmPin, 0); // instead of analogWrite
    }

    // testing stuff
    Serial.printf("PWM %d | CW %d | CCW %d | Speed %d\n", pwmPin,
                  digitalRead(clockwiseDirPin), digitalRead(antiClockwiseDirPin), speed);
}

// Arduino setup function. Runs in CPU 1
void setup()
{
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t *addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);

    // motor contoller pins configuration initialize
    pinMode(leftMotorClockwiseDirPin, OUTPUT);
    pinMode(leftMotorAntiClockwiseDirPin, OUTPUT);
    pinMode(rightMotorClockwiseDirPin, OUTPUT);
    pinMode(rightMotorAntiClockwiseDirPin, OUTPUT);

    // // PWM pin setup for ledc version 3.*
    // ledcAttach(leftMotorPWMPin, 1000, 8); // Attach PWM @1kHz, 8-bit res
    // ledcAttach(rightMotorPWMPin, 1000, 8); // Attach PWM @1kHz, 8-bit res

    // PWM pin setup for ledc older version 2.*
    ledcSetup(LEFT_MOTOR_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(leftMotorPWMPin, LEFT_MOTOR_CHANNEL);

    ledcSetup(RIGHT_MOTOR_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(rightMotorPWMPin, RIGHT_MOTOR_CHANNEL);

    Serial.println("Ready to move!");
}

// Arduino loop function. Runs in CPU 1.
void loop()
{
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();
    else
    {
        setMotorV2(LEFT_MOTOR_CHANNEL, leftMotorClockwiseDirPin, leftMotorAntiClockwiseDirPin, 0);
        setMotorV2(RIGHT_MOTOR_CHANNEL, rightMotorClockwiseDirPin, rightMotorAntiClockwiseDirPin, 0);
    }

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    vTaskDelay(1);
    delay(150);
}
