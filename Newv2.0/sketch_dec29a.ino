/* * ======================================================================================
 * PROJECT:      ACTIVA HYPER-SPORT SMART KEY SYSTEM
 * VERSION:      4.0 (TITANIUM EDITION)
 * ARCHITECTURE: NON-BLOCKING ASYNCHRONOUS STATE MACHINE
 * AUTHOR:       GEMINI (AI) & USER
 * DATE:         JANUARY 2026
 * ======================================================================================
 * * --- HARDWARE REQUIREMENTS ---
 * 1. PROCESSOR:   Arduino Nano (ATmega328P)
 * 2. BLUETOOTH:   HC-05 Module (Baud 9600)
 * 3. RELAYS:      4-Channel Relay Module (Active LOW Trigger)
 * 4. POWER:       5V Regulated Source (Must have 1000uF Capacitor across 5V/GND)
 * * --- WIRING MAP ---
 * [D2]  <-- PHYSICAL BUTTON (Connect other side to GND)
 * [D3]  --> RELAY 1: IGNITION / FI SYSTEM
 * [D4]  --> RELAY 2: STARTER MOTOR
 * [D5]  --> STATUS LED (Dashboard)
 * [D6]  <-- HC-05 'STATE' PIN (Critical for Auto-Connect Animation)
 * [D7]  --> RELAY 3: HORN
 * [D8]  --> RELAY 4: INDICATORS (Left + Right Combined)
 * [D10] <-- HC-05 TX PIN
 * [D11] --> HC-05 RX PIN
 * [D12] --> TRANSISTOR BASE (For Cutting BT Power in Sleep)
 * [A0]  <-- VOLTAGE SENSOR (30k + 7.5k Resistor Divider)
 * * ======================================================================================
 */

#include <SoftwareSerial.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <EEPROM.h> 

// ==================================================
//               1. SYSTEM CONFIGURATION
// ==================================================

// --- TIMING SETTINGS ---
const unsigned long AUTO_SLEEP_TIMEOUT = 1200000; // 20 Minutes (in ms)
const unsigned long HEARTBEAT_INTERVAL = 2000;    // 2 Seconds (App Keep-Alive)
const unsigned long BATTERY_CHECK_INTERVAL = 10000; // 10 Seconds
const unsigned long CRANK_LIMIT_MS     = 2800;    // Max Starter Crank Time
const unsigned long FUEL_PUMP_PRIME    = 1500;    // Fuel Pump Prime Time
const unsigned long DEBOUNCE_DELAY     = 60;      // Button Debounce

// --- VOLTAGE CALIBRATION ---
// Formula: Vout = (Vin * R2) / (R1 + R2)
const float RESISTOR_R1 = 30000.0; // 30k Ohm
const float RESISTOR_R2 = 7500.0;  // 7.5k Ohm
const float REF_VOLTAGE = 5.0;     // Arduino VCC

// --- EEPROM ADDRESSES ---
const int ADDR_LOCK_STATUS = 0;    // Memory slot 0 for Lock State

// ==================================================
//               2. GLOBAL VARIABLES
// ==================================================

// --- PIN DEFINITIONS ---
#define PIN_BUTTON      2
#define PIN_IGNITION    3
#define PIN_STARTER     4
#define PIN_LED         5
#define PIN_BT_STATE    6
#define PIN_HORN        7
#define PIN_INDICATOR   8
#define PIN_RX          10
#define PIN_TX          11
#define PIN_BT_PWR      12
#define PIN_VOLT        A0

// --- COMMUNICATION ---
SoftwareSerial BTSerial(PIN_RX, PIN_TX);

// --- SYSTEM FLAGS ---
bool isEngineRunning = false;
bool isLocked = false;
bool isBtConnected = false;
bool isDeepSleepPending = false;

// --- TIMER VARIABLES ---
unsigned long lastLoopTime = 0;
unsigned long sleepTimer = 0;
unsigned long heartbeatTimer = 0;
unsigned long batteryTimer = 0;

// --- BUTTON LOGIC ---
int buttonState = HIGH;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long buttonPressTime = 0;
bool buttonActionHandled = false;
int multiTapCounter = 0;
unsigned long lastTapTime = 0;

// --- ANIMATION ENGINE ---
int animMode = 0;       // 0=Idle, 1=Welcome, 2=Find, 3=Hazard, 4=Lock
int animStep = 0;
unsigned long animTimer = 0;
int animLoops = 0;

// ==================================================
//                  3. BOOT SEQUENCE
// ==================================================

void setup() {
  // 1. DISABLE INTERRUPTS DURING SETUP
  noInterrupts();
  
  // 2. CONFIGURE INPUTS
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BT_STATE, INPUT);
  
  // 3. CONFIGURE RELAYS (SAFETY FIRST: SET HIGH IMMEDIATELY)
  // Most Relay Modules are Active LOW. We write HIGH to ensure they stay OFF.
  pinMode(PIN_IGNITION, OUTPUT); digitalWrite(PIN_IGNITION, HIGH); 
  pinMode(PIN_STARTER, OUTPUT);  digitalWrite(PIN_STARTER, HIGH);
  pinMode(PIN_HORN, OUTPUT);     digitalWrite(PIN_HORN, HIGH);
  pinMode(PIN_INDICATOR, OUTPUT);digitalWrite(PIN_INDICATOR, HIGH);
  
  // 4. CONFIGURE AUXILIARY
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BT_PWR, OUTPUT);   digitalWrite(PIN_BT_PWR, HIGH); // Enable Bluetooth Power
  
  // 5. START SERIAL PORTS
  Serial.begin(9600);   // USB Debugging
  BTSerial.begin(9600); // HC-05 Communication
  
  // 6. RESTORE SYSTEM STATE FROM MEMORY
  byte savedLockState = EEPROM.read(ADDR_LOCK_STATUS);
  if (savedLockState == 1) {
    isLocked = true;
    Serial.println(F("BOOT: System Restored in LOCKED Mode"));
  } else {
    isLocked = false;
    Serial.println(F("BOOT: System Restored in UNLOCKED Mode"));
  }

  // 7. ENABLE INTERRUPTS & RESET TIMERS
  interrupts();
  sleepTimer = millis();
  
  // 8. VISUAL CONFIRMATION (System Ready)
  digitalWrite(PIN_LED, HIGH);
  delay(200);
  digitalWrite(PIN_LED, LOW);
  
  Serial.println(F("SYSTEM READY. WAITING FOR COMMANDS..."));
}

// ==================================================
//                  4. MAIN KERNEL
// ==================================================

void loop() {
  unsigned long currentMillis = millis();
  
  // --- LAYER 1: HARDWARE INPUTS ---
  scanPhysicalButton();    // Check for user clicks
  checkBluetoothStatus();  // Check if phone connected (Pin 6)
  readBluetoothCommands(); // Parse incoming data
  
  // --- LAYER 2: BACKGROUND PROCESSES ---
  processAnimations();     // Handle blinking lights
  sendHeartbeat(currentMillis); // Keep App alive
  checkBatteryHealth(currentMillis); // Monitor Voltage
  
  // --- LAYER 3: POWER MANAGEMENT ---
  // If engine is off, and no buttons pressed for 20 mins -> Sleep
  if (!isEngineRunning && (currentMillis - sleepTimer > AUTO_SLEEP_TIMEOUT)) {
    initiateDeepSleep();
  }
}

// ==================================================
//               5. INPUT HANDLERS
// ==================================================

void checkBluetoothStatus() {
  // Reads the "STATE" pin of HC-05. HIGH = Connected. LOW = Searching.
  bool currentState = digitalRead(PIN_BT_STATE);
  
  if (currentState && !isBtConnected) {
    // EVENT: Just Connected
    isBtConnected = true;
    sleepTimer = millis(); // Reset sleep timer
    triggerAnimation(1);   // Play "Welcome" Pulse
    Serial.println(F("EVENT: Phone Connected"));
  } 
  else if (!currentState && isBtConnected) {
    // EVENT: Connection Lost
    isBtConnected = false;
    Serial.println(F("EVENT: Phone Disconnected"));
    // Note: We do NOT kill the engine here for safety.
  }
}

void readBluetoothCommands() {
  if (BTSerial.available()) {
    sleepTimer = millis(); // Activity detected
    String command = BTSerial.readStringUntil('\n');
    command.trim(); // Remove \r \n whitespace
    
    if (command.length() == 0) return;
    
    Serial.print(F("CMD RX: ")); Serial.println(command);
    
    // Command Routing
    if (command == "START")       sequenceStartEngine();
    else if (command == "STOP")   sequenceStopEngine();
    else if (command == "FIND")   triggerAnimation(2);
    else if (command == "LOCK")   toggleLockSystem(true);
    else if (command == "UNLOCK") toggleLockSystem(false);
  }
}

void scanPhysicalButton() {
  int reading = digitalRead(PIN_BUTTON);
  
  // Debounce Logic
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) { 
        // EVENT: Button Pressed Down
        buttonPressTime = millis();
        buttonActionHandled = false;
        sleepTimer = millis();
      } 
      else { 
        // EVENT: Button Released
        unsigned long pressDuration = millis() - buttonPressTime;
        
        if (!buttonActionHandled) {
          executeButtonLogic(pressDuration);
          buttonActionHandled = true;
        }
      }
    }
  }
  lastButtonState = reading;
}

void executeButtonLogic(unsigned long duration) {
  // Security Check
  if (isLocked) {
    // If locked, reject action and flash LED aggressively
    Serial.println(F("ACTION DENIED: System Locked"));
    for(int i=0; i<3; i++) {
       digitalWrite(PIN_LED, HIGH); delay(50); digitalWrite(PIN_LED, LOW); delay(50);
    }
    return;
  }

  // Action Router
  if (duration > 1000) { 
    // --- LONG PRESS (> 1s) ---
    // Acts as Master Switch: Starts or Stops engine
    if (isEngineRunning) {
      sequenceStopEngine();
    } else {
      sequenceStartEngine();
    }
  } 
  else {
    // --- SHORT TAP (< 1s) ---
    // Maintenance Mode: Toggles Ignition Only (No Crank)
    // Useful for checking fuel level without starting engine
    if (!isEngineRunning) {
       Serial.println(F("ACTION: Toggle Ignition"));
       bool currentRelayState = digitalRead(PIN_IGNITION); // Remember LOW is ON
       
       if (currentRelayState == HIGH) {
         digitalWrite(PIN_IGNITION, LOW); // Turn ON
         triggerAnimation(3); // Small Chirp
       } else {
         digitalWrite(PIN_IGNITION, HIGH); // Turn OFF
       }
    }
  }
}

// ==================================================
//               6. ACTION SEQUENCES
// ==================================================

void sequenceStartEngine() {
  if (isEngineRunning) return;
  if (isLocked) return;
  
  Serial.println(F("SEQ: Initiating Start..."));
  
  // PHASE 1: IGNITION ON
  digitalWrite(PIN_IGNITION, LOW); // Active LOW -> ON
  
  // PHASE 2: VISUAL WARNING (2 Flashes)
  digitalWrite(PIN_INDICATOR, LOW); delay(100); digitalWrite(PIN_INDICATOR, HIGH); delay(100);
  digitalWrite(PIN_INDICATOR, LOW); delay(100); digitalWrite(PIN_INDICATOR, HIGH);
  
  // PHASE 3: PRIME FUEL PUMP
  delay(FUEL_PUMP_PRIME);
  
  // PHASE 4: CRANK STARTER
  Serial.println(F("SEQ: Cranking Starter..."));
  digitalWrite(PIN_STARTER, LOW);  // Engage Starter
  delay(CRANK_LIMIT_MS);           // Wait 2.8s
  digitalWrite(PIN_STARTER, HIGH); // Disengage Starter
  
  // PHASE 5: CONFIRMATION
  isEngineRunning = true;
  BTSerial.println("ENGINE_ON");
  Serial.println(F("SEQ: Engine Running"));
}

void sequenceStopEngine() {
  Serial.println(F("SEQ: Stopping Engine..."));
  
  // 1. Visual Confirmation (Long Flash)
  digitalWrite(PIN_INDICATOR, LOW); delay(400); digitalWrite(PIN_INDICATOR, HIGH);
  
  // 2. Kill Ignition
  digitalWrite(PIN_IGNITION, HIGH); // Active LOW -> OFF
  
  isEngineRunning = false;
  BTSerial.println("ENGINE_OFF");
  Serial.println(F("SEQ: Engine Stopped"));
}

void toggleLockSystem(bool lock) {
  isLocked = lock;
  
  // Save state to EEPROM (Permanent Memory)
  EEPROM.write(ADDR_LOCK_STATUS, isLocked ? 1 : 0);
  
  if (isLocked) {
    // --- LOCKING SEQUENCE ---
    Serial.println(F("SYS: LOCKED"));
    
    // Kill engine if running
    if (isEngineRunning) sequenceStopEngine();
    
    // Visual/Audio: One Long Beep/Flash
    digitalWrite(PIN_INDICATOR, LOW); digitalWrite(PIN_HORN, LOW);
    delay(600);
    digitalWrite(PIN_INDICATOR, HIGH); digitalWrite(PIN_HORN, HIGH);
    
    // Force Sleep to save battery
    initiateDeepSleep();
  } 
  else {
    // --- UNLOCKING SEQUENCE ---
    Serial.println(F("SYS: UNLOCKED"));
    
    // Visual/Audio: Two Short Beeps/Flashes
    for(int i=0; i<2; i++) {
      digitalWrite(PIN_INDICATOR, LOW); digitalWrite(PIN_HORN, LOW); delay(100);
      digitalWrite(PIN_INDICATOR, HIGH); digitalWrite(PIN_HORN, HIGH); delay(100);
    }
  }
}

// ==================================================
//             7. ANIMATION CONTROLLER
// ==================================================

void triggerAnimation(int mode) {
  animMode = mode;
  animStep = 0;
  animLoops = 0;
  animTimer = millis();
}

void processAnimations() {
  if (animMode == 0) return; // No animation active
  
  unsigned long now = millis();
  
  // --- MODE 1: WELCOME (The "Heartbeat" Pulse) ---
  if (animMode == 1) {
    if (animStep == 0) {
      digitalWrite(PIN_INDICATOR, LOW); // ON
      animTimer = now; animStep = 1;
    }
    else if (animStep == 1 && (now - animTimer > 150)) {
      digitalWrite(PIN_INDICATOR, HIGH); // OFF
      animTimer = now; animStep = 2;
    }
    else if (animStep == 2 && (now - animTimer > 100)) {
      digitalWrite(PIN_INDICATOR, LOW); // ON
      animTimer = now; animStep = 3;
    }
    else if (animStep == 3 && (now - animTimer > 350)) {
      digitalWrite(PIN_INDICATOR, HIGH); // OFF
      animTimer = now; animStep = 4;
    }
    else if (animStep == 4 && (now - animTimer > 400)) {
      animLoops++;
      if (animLoops >= 3) { animMode = 0; } // End after 3 cycles
      else { animStep = 0; } // Repeat
    }
  }
  
  // --- MODE 2: FIND ME (Flash + Horn) ---
  else if (animMode == 2) {
    if (animStep == 0) {
      digitalWrite(PIN_INDICATOR, LOW); digitalWrite(PIN_HORN, LOW); // BOTH ON
      animTimer = now; animStep = 1;
    }
    else if (animStep == 1 && (now - animTimer > 300)) {
      digitalWrite(PIN_INDICATOR, HIGH); digitalWrite(PIN_HORN, HIGH); // BOTH OFF
      animTimer = now; animStep = 2;
    }
    else if (animStep == 2 && (now - animTimer > 300)) {
      animLoops++;
      if (animLoops >= 5) { animMode = 0; } // End after 5 cycles
      else { animStep = 0; }
    }
  }
  
  // --- MODE 3: SHORT CHIRP (Confirmation) ---
  else if (animMode == 3) {
    if (animStep == 0) {
      digitalWrite(PIN_INDICATOR, LOW); 
      animTimer = now; animStep = 1;
    }
    else if (animStep == 1 && (now - animTimer > 50)) {
      digitalWrite(PIN_INDICATOR, HIGH); 
      animMode = 0; // End immediately
    }
  }
}

// ==================================================
//            8. TELEMETRY & HEARTBEAT
// ==================================================

void sendHeartbeat(unsigned long now) {
  // This function keeps the MIT App Connected (Green Status)
  if (now - heartbeatTimer > HEARTBEAT_INTERVAL) {
    heartbeatTimer = now;
    
    // Broadcast Status
    if (isEngineRunning) {
      BTSerial.println("ENGINE_ON");
    } else {
      BTSerial.println("ENGINE_OFF");
    }
    
    // Optional: Send Lock Status for Debug
    // if(isLocked) Serial.println("STATUS: Locked");
  }
}

void checkBatteryHealth(unsigned long now) {
  // Reads voltage divider to monitor battery health
  if (now - batteryTimer > BATTERY_CHECK_INTERVAL) {
    batteryTimer = now;
    
    int sensorValue = analogRead(PIN_VOLT);
    float voltage = sensorValue * (REF_VOLTAGE / 1024.0);
    float inputVoltage = voltage / (RESISTOR_R2 / (RESISTOR_R1 + RESISTOR_R2));
    
    // Send data to Serial for debugging (Optional: send to App if needed)
    // Format: VOLT:12.4
    // Serial.print("BATTERY: "); Serial.print(inputVoltage); Serial.println("V");
  }
}

// ==================================================
//             9. DEEP SLEEP MANAGER
// ==================================================

void wakeUpInterrupt() {
  // ISR: Minimal code here. Just wakes the CPU.
}

void initiateDeepSleep() {
  Serial.println(F("PWR: Entering Deep Sleep Mode..."));
  delay(100); // Allow Serial buffer to empty
  
  // 1. SHUTDOWN PERIPHERALS
  digitalWrite(PIN_BT_PWR, LOW);   // Cut power to HC-05
  digitalWrite(PIN_LED, LOW);      // Turn off Status LED
  
  // 2. CONFIGURE SLEEP PARAMETERS
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Lowest power mode
  sleep_enable();
  
  // 3. ARM WAKE-UP INTERRUPT
  // We attach interrupt to the Button Pin (D2)
  // When button goes LOW (Pressed), system wakes up.
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), wakeUpInterrupt, LOW);
  
  // 4. ENTER SLEEP
  sleep_mode(); 
  
  // ====================================================
  //               SYSTEM SLEEPS HERE
  // ====================================================
  
  // 5. WAKE UP SEQUENCE (Code resumes here after button press)
  sleep_disable(); // Prevent accidental sleep
  detachInterrupt(digitalPinToInterrupt(PIN_BUTTON)); // Clean up interrupt
  
  // 6. RESTORE POWER
  digitalWrite(PIN_BT_PWR, HIGH); // Restore Bluetooth
  
  // 7. WAKE CONFIRMATION
  Serial.println(F("PWR: Waking Up!"));
  digitalWrite(PIN_LED, HIGH); delay(200); digitalWrite(PIN_LED, LOW);
  
  // 8. RESET TIMERS
  sleepTimer = millis(); 
  isBtConnected = false; // Force re-check of BT connection
}