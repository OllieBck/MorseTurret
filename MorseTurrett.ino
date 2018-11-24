/*********************************************************************
  This is an example for our nRF52 based Bluefruit LE modules

  Pick one up today in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <bluefruit.h>


// BLE Service
BLEDis  bledis;
//BLEUart bleuart;
BLEBas  blebas;

BLEService morseControl = BLEService(0x1234);
BLECharacteristic morseChar = BLECharacteristic(0xABCD);
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *morseMotor = AFMS.getStepper(200, 2);

int previousLetter = 'a';
int previousNumber = '0';
int aPin = 16;
int rotationCount = 0;
bool motorCalibrated = false;

// for testing
int countMotor = 0;

const int relayPin = 7;

// Software Timer for blinking RED LED
SoftwareTimer blinkTimer;

void setup()
{
  pinMode(relayPin, OUTPUT);
  pinMode(aPin, INPUT_PULLUP);
  Serial.begin(115200);

  // Initialize blinkTimer for 1000 ms and start it
  blinkTimer.begin(1000, blink_timer_callback);
  blinkTimer.start();

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.setName("MorseControl");
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  //bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Start custom BLE Service
  morseControl.begin();

  // Start custom Characteristic
  morseChar.setProperties(CHR_PROPS_WRITE);
  morseChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  morseChar.setFixedLen(1); // Alternatively .setMaxLen(uint16_t len)
  morseChar.begin();

  // Set up and start advertising
  startAdv();
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  morseMotor->setSpeed(50);  // 10 rpm
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  // Bluefruit.Advertising.addService(bleuart);

  // Advertise the custom service
  Bluefruit.Advertising.addService(morseControl);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)

     For recommended advertising interval
     https://developer.apple.com/library/content/qa/qa1931/_index.html
  */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void loop()
{
  while (motorCalibrated == false){
    motorCalibrated = calibrateMotor();
  }
  
  
  int currentLetter = morseChar.read8();
  if (currentLetter >= 97 && currentLetter <= 122) {
    digitalWrite(relayPin, LOW);
    int addLetters = (previousLetter - currentLetter) * 330;
    Serial.print("current letter: ");
    Serial.print(currentLetter);
    Serial.print(" previous letter: ");
    Serial.print(previousLetter);
    Serial.print(" calc letters: ");
    Serial.println(addLetters);
    if (addLetters < 0) {
      morseMotor->step(abs(addLetters), FORWARD, SINGLE);
      morseMotor->release();
    }
    else if (addLetters > 0) {
      morseMotor->step(abs(addLetters), BACKWARD, SINGLE);
      morseMotor->release();
    }
    else {

    }
    previousLetter = currentLetter;
  }
  else if (currentLetter >= 48 && currentLetter <= 57) {
    if (currentLetter != previousNumber) {
      digitalWrite(relayPin, HIGH);
      delay(5000);
      digitalWrite(relayPin, LOW);
      previousNumber = currentLetter;
    }
  }
  else {
    digitalWrite(relayPin, LOW);
  }
  delay(2000);



  // Request CPU to enter low-power mode until an event/interrupt occurs
  waitForEvent();
}

bool calibrateMotor() {
  if (digitalRead(aPin) == LOW) {
    motorCalibrated = true;
    morseMotor->step(10, FORWARD, DOUBLE);
    morseMotor->release();
    previousLetter = 'a';
    return true;
  }
  morseMotor->step(50, BACKWARD, DOUBLE);
  rotationCount++;
  Serial.print("rotation: ");
  Serial.println(rotationCount);
}



void connect_callback(uint16_t conn_handle)
{
  char central_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.println("Disconnected");
}

/**
   Software Timer callback is invoked via a built-in FreeRTOS thread with
   minimal stack size. Therefore it should be as simple as possible. If
   a periodically heavy task is needed, please use Scheduler.startLoop() to
   create a dedicated task for it.

   More information http://www.freertos.org/RTOS-software-timer.html
*/
void blink_timer_callback(TimerHandle_t xTimerID)
{
  (void) xTimerID;
  digitalToggle(LED_RED);
}

/**
   RTOS Idle callback is automatically invoked by FreeRTOS
   when there are no active threads. E.g when loop() calls delay() and
   there is no bluetooth or hw event. This is the ideal place to handle
   background data.

   NOTE: FreeRTOS is configured as tickless idle mode. After this callback
   is executed, if there is time, freeRTOS kernel will go into low power mode.
   Therefore waitForEvent() should not be called in this callback.
   http://www.freertos.org/low-power-tickless-rtos.html

   WARNING: This function MUST NOT call any blocking FreeRTOS API
   such as delay(), xSemaphoreTake() etc ... for more information
   http://www.freertos.org/a00016.html
*/
void rtos_idle_callback(void)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here
}
