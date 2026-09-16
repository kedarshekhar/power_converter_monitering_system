#include <SPI.h>               // Library to talk to the display over SPI (fast serial bus)
#include <Adafruit_GFX.h>      // Base graphics library (shapes, text, etc.)
#include <Adafruit_ST7789.h>   // Driver library specific to the ST7789 TFT display
#include <math.h>              // Gives us math functions like sin()

// =========================================================================
// ================= ETHERNET LINK (USR-TCP232 SERIAL<->ETHERNET) =========
// WiFi has been replaced with a wired link for speed/reliability. The
// USR-TCP232 module (STM32H750 + RJ45, seen on your board) is a
// transparent UART<->TCP bridge: any byte we write out ETH_TXD comes out
// the far end as a TCP byte to the PC, and any TCP byte the PC sends
// arrives on ETH_RXD. From the ESP32's point of view it's "just a serial
// port" - all the Ethernet/TCP handling is done by the module's own
// STM32, not by us.
//
// IMPORTANT - one-time module configuration (NOT done in this sketch):
// Use USR's "USR-VCOM" / web-config tool (via the module's own IP,
// factory default usually 192.168.0.7) to set:
//   - Work Mode: TCP Server (so the PC/Python GUI connects TO it)
//   - Local Port: e.g. 8899
//   - A static IP on your LAN (or note the DHCP-assigned one)
//   - Serial params: 115200 baud, 8 data bits, No parity, 1 stop bit
//     (must match ETH_BAUD below)
// =========================================================================
#define ETH_RXD 16   // ESP32 RX  <- wire to module's TXD1/TXD pad
#define ETH_TXD 17   // ESP32 TX  -> wire to module's RXD1/RXD pad
#define ETH_BAUD 256000

HardwareSerial EthSerial(1);   // Use ESP32's UART1 peripheral for the Ethernet bridge

String ethRxLineBuffer = "";   // Accumulates incoming bytes from the module until a newline

// ================= PHYSICAL BUTTON PINS =================
#define BTN_ONOFF    33  // 1st Button: System ON/OFF          -> GPIO33
#define BTN_WAVE     32  // 2nd Button: Switch Page/Waveform    -> GPIO32
#define BTN_INC      25  // 3rd Button: Increment Digit         -> GPIO25
#define BTN_DEC      26  // 4th Button: Decrement Digit         -> GPIO26
#define BTN_SAVE     27  // 5th Button: Save / Enter            -> GPIO27
#define BTN_CURSOR   14  // 6th Button: Move Cursor              -> GPIO14

// ================= TFT PINS =================
#define TFT_CS   5   // Chip Select pin - tells the display "I'm talking to you now"
#define TFT_DC   2   // Data/Command pin - tells display if incoming byte is a command or data
#define TFT_RST  4   // Reset pin - used to reboot the display chip

// ================= VSPI HARDWARE PINS (ESP32) =================
#define VSPI_MOSI 23  // Master Out Slave In - ESP32 sends data to display on this pin
#define VSPI_MISO 19  // Master In Slave Out - display sends data back to ESP32 (rarely used here)
#define SPI_SCK  18   // Clock pin - keeps ESP32 and display in sync while sending bits

// ================= UART2 PINS (FOR PUTTY / SCI PORT 2) =================
#define RXD2 21          // Default ESP32 RX2 Pin (ESP32 receives data here)
#define TXD2 22          // Default ESP32 TX2 Pin (ESP32 sends data here, e.g. to PuTTY)

// ================= SCREEN DIMENSIONS (PORTRAIT) =================
#define SCREEN_W 240   // Screen width in pixels
#define SCREEN_H 320   // Screen height in pixels

// ================= HARDWARE-MAPPED COLOR MATRIX =================
// These are 16-bit RGB565 color codes (a compact way to store colors for TFT screens)
#define COLOR_BG      0x0000  // Pure Black
#define COLOR_HEADER  0x10A4  // Deep charcoal blue
#define COLOR_TEXT    0xFFFF  // Crisp white
#define COLOR_LABEL   0x9CF3  // Light muted blue/grey
#define COLOR_BORDER  0x5AEB  // Premium sleek mid-grey for frames
#define COLOR_CURSOR  0xFCE0  // Bright Yellow Accent for Cursor
#define COLOR_GRID    0x2124  // Grid lines
#define COLOR_AXIS    0x7BEF  // Light grey for graph axes

// Dynamic color routing based on your panel's hardware response:
#define COLOR_OUTPUT  0x07E0  // Physical Green (used for waveform graphs)
#define COLOR_ON      0x07E0  // Physical Green

// ================= ON/OFF STATUS HIGHLIGHT COLORS =================
#define HIGHLIGHT_ON   0xF81F  // Vivid Magenta (was Power color) for ON button
#define HIGHLIGHT_OFF  0x07FF  // Electric Cyan (was Voltage color) for OFF button

// ================= APPEALING VALUE TEXT COLOR PALETTE =================
#define COLOR_VOLTAGE  0x07FF  // Electric Cyan for Voltage readout
#define COLOR_CURRENT  0xFD20  // Warm Amber/Gold for Current readout
#define COLOR_POWER    0x7A95  // Royal Purple for Power readout
#define COLOR_SET_BADGE 0xFC18 // Hot Pink for SET! badge

// Create the display object, telling it which pins to use for CS, DC, RST
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ================= PAGE STATE =================
// This enum lists the different "screens"/pages the user can flip through
enum ScreenPage {
  PAGE_DASHBOARD,  // Main screen showing voltage/current/power
  PAGE_VOLTAGE,    // Voltage waveform screen
  PAGE_CURRENT,    // Current waveform screen
  PAGE_POWER,      // Power waveform screen
  PAGE_TERMINAL,   // PuTTY terminal display screen
  PAGE_C2000       // Data received from C2000 board
};

ScreenPage currentPage = PAGE_DASHBOARD;  // Start on the dashboard page

// ================= VARIABLES =================
bool outputActive = false;      // Is the converter output currently ON or OFF?

float inputVoltage = 24.0;      // Simulated input voltage to the buck converter
float outputVoltage = 0.0;      // Calculated/simulated output voltage
float outputCurrent = 0.0;      // Simulated output current
float outputPower = 0.0;        // Calculated output power (V * I)
float dutyCycle = 0.0;          // PWM duty cycle as a percentage (0-100)

float vRef = 0.0;               // Target reference voltage set by the user, Initialized to 0.0 V
float pwmDuty = 0.0;            // PWM duty cycle as a fraction (0.0-1.0)

// Initialized to 00.00 format always
int vRef0 = 0;   // Tens digit of Vref
int vRef1 = 0;   // Ones digit of Vref
int vRef2 = 0;   // First decimal digit of Vref
int vRef3 = 0;   // Second decimal digit of Vref
int cursorIndex = 0;  // Which digit (0-3) is currently selected for editing

bool showSetNotification = false;      // Should we show the "SET!" badge right now?
unsigned long setNotificationTimer = 0; // Timestamp of when SET badge was triggered

// ================= SERIAL TERMINAL DISPLAY (PUTTY) =================
#define TERM_MAX_LINES 12      // How many lines fit on screen
#define TERM_MAX_CHARS 34      // Max characters per line before auto-wrap
String termLines[TERM_MAX_LINES];   // Circular buffer of received lines
int termLineCount = 0;               // How many lines currently used
String termInputBuffer = "";         // Currently-being-typed line (not yet Enter-terminated)

// ================= C2000 RECEIVE DATA =================
float c2000Voltage = 0.0;           // Last voltage value received from the C2000 (header byte 1)
float c2000Current = 0.0;           // Last current value received from the C2000 (header byte 2)
float c2000Power   = 0.0;           // Last power value received from the C2000 (header byte 3)
bool c2000VoltageReceived = false;  // Have we received at least one voltage packet?
bool c2000CurrentReceived = false;  // Have we received at least one current packet?
bool c2000PowerReceived   = false;  // Have we received at least one power packet?
unsigned long c2000LastRxTime = 0;  // millis() timestamp of the last successful packet (any type)

String c2000RxLineBuffer = "";      // Buffer for building up one "0xXX" text line
uint8_t c2000RxPacket[6];           // Holds the 6 bytes of the packet currently being assembled
int c2000RxByteIndex = 0;           // Which byte of the packet we're filling next

// ================= BUTTON STRUCT =================
// A "struct" bundles related variables together - here, everything needed to debounce one button
struct Button {
  uint8_t pin;              // Which GPIO pin this button is on
  bool lastState;           // The last raw reading we saw from the pin
  unsigned long debounceTime; // Timestamp used to filter out electrical noise/bounce
  bool pressed;              // Whether we've already registered this press (to avoid repeats)
};

// Array holding all 6 buttons, each starting HIGH (not pressed, due to INPUT_PULLUP)
Button buttons[] = {
  {BTN_ONOFF,   HIGH, 0, false},
  {BTN_WAVE,    HIGH, 0, false},
  {BTN_INC,     HIGH, 0, false},
  {BTN_DEC,     HIGH, 0, false},
  {BTN_SAVE,    HIGH, 0, false},
  {BTN_CURSOR,  HIGH, 0, false}
};

// ================= FUNCTION PROTOTYPES =================
// Declaring functions ahead of time so the compiler knows they exist before we define them below
void clearEntireScreen();
void drawOuterBorder();
void drawDashboard();
void updateValues();
void drawVref();
void drawGraphAxes();
void drawVoltagePage();
void drawCurrentPage();
void drawPowerPage();
void drawStatusArea();
void handleButtonPress(int pin);
void checkButtons();
float calculateOutputVoltage(float vin, float duty);
void updateBuckParameters();
void floatToLE4Bytes(float value, uint8_t outBytes[4]);
void sendByteAsHex(uint8_t value);
void sendOnSignalPacket();
void sendOffSignalPacket();
void sendVrefPacket();
void drawTerminalPage();
void checkPuttyInput();
void addTerminalLine(String line);
void checkC2000Input();
void handleC2000Packet(uint8_t *pkt);
void drawC2000Page();
void setupEthernetLink();
void checkEthernetInput();
void handleGuiCommand(String cmd);
void broadcastEthernetData();

// ================= BUCK CONVERTER CALCULATIONS =================
// Simple formula: output voltage = input voltage * duty cycle * efficiency
float calculateOutputVoltage(float vin, float duty) {
  float efficiency = 0.92;               // Assume 92% efficient conversion
  return vin * duty * efficiency;        // Return the calculated output voltage
}

// Recalculates all the buck converter's numbers based on current Vref and input voltage
void updateBuckParameters() {
  // Combine the 4 separate digits into one decimal number, e.g. 1,2,3,4 -> 12.34
  vRef = vRef0 * 10.0 + vRef1 * 1.0 + vRef2 * 0.1 + vRef3 * 0.01;
  
  if (inputVoltage > 0) {
    pwmDuty = vRef / inputVoltage;       // Duty cycle needed to hit the target voltage
    if (pwmDuty > 0.95) pwmDuty = 0.95;  // Clamp so duty cycle never exceeds 95%
    if (pwmDuty < 0.05) pwmDuty = 0.05;  // Clamp so duty cycle never goes below 5%
  } else {
    pwmDuty = 0.5;                       // Fallback if input voltage is invalid (0 or negative)
  }
  
  if (outputActive) {
    outputVoltage = calculateOutputVoltage(inputVoltage, pwmDuty);   // Compute output voltage
    outputCurrent = 1.5 + (random(0, 100) / 100.0) * 2.0;            // Simulate a random current 1.5-3.5A
    outputPower = outputVoltage * outputCurrent;                      // Power = Voltage * Current
    dutyCycle = pwmDuty * 100.0;                                      // Convert fraction to percentage
  } else {
    // If output is off, everything reads zero
    outputVoltage = 0.0;
    outputCurrent = 0.0;
    outputPower = 0.0;
    dutyCycle = 0.0;
  }
}

// Fills the whole screen with the background color (like erasing a whiteboard)
void clearEntireScreen() {
  tft.fillScreen(COLOR_BG);
}

// ================= DRAW 4-PIXEL OUTER BORDER =================
// Draws 4 nested rectangles to create a thick border effect around the screen edge
void drawOuterBorder() {
  tft.drawRect(0, 0, SCREEN_W, SCREEN_H, COLOR_BORDER);          // Outermost rectangle
  tft.drawRect(1, 1, SCREEN_W - 2, SCREEN_H - 2, COLOR_BORDER);  // 1px inward
  tft.drawRect(2, 2, SCREEN_W - 4, SCREEN_H - 4, COLOR_BORDER);  // 2px inward
  tft.drawRect(3, 3, SCREEN_W - 6, SCREEN_H - 6, COLOR_BORDER);  // 3px inward
}

// =========================================================================
// ================= PROPER IEEE-754 FLOAT -> 4-BYTE LE CONVERSION ========
// =========================================================================
// Uses a union to reinterpret the float's raw memory as 4 bytes, preserving
// the EXACT value (fractional part included) with full IEEE-754 precision.
// ESP32 is natively little-endian, so byte[0] of the union is already the
// LSB of the float's in-memory representation — no manual swap needed.
// =========================================================================
void floatToLE4Bytes(float value, uint8_t outBytes[4]) {
  // A "union" lets the same 4 bytes of memory be viewed either as one float
  // or as 4 separate bytes - this is how we "peek inside" the float's bits
  union {
    float f;      // View this memory as a float
    uint8_t b[4]; // View this memory as 4 raw bytes
  } converter;

  converter.f = value;   // Store the float value; this also fills converter.b[] with its raw bytes

  outBytes[0] = converter.b[0]; // LSB (least significant byte, sent first)
  outBytes[1] = converter.b[1]; // 2nd byte
  outBytes[2] = converter.b[2]; // 3rd byte
  outBytes[3] = converter.b[3]; // MSB (most significant byte, sent last)
}

// =========================================================================
// Sends a single byte to Serial2 (PuTTY) and Serial (USB), each on its own
// line, formatted as "0xXX". Called individually per byte.
// =========================================================================
void sendByteAsHex(uint8_t value) {
  char buf[6];                                   // Small text buffer to hold formatted string
  snprintf(buf, sizeof(buf), "0x%02X", value);    // Format the byte as hex text, e.g. "0xA1"
  Serial2.println(buf);                           // Send it out over UART2 (to PuTTY)
  Serial.println(buf);                             // Also print it on USB serial (for debugging)
}

// =========================================================================
// ================= DEDICATED ON-TRANSITION SIGNAL PACKET ================
// Sent ONLY at the exact moment the output is switched ON.
//   byte[0..4] = 0x11, 0x11, 0x11, 0x11, 0x11   (first 5 bytes)
//   byte[5]    = 0x00                            (last byte)
// The current Vref value is sent right after it via sendVrefPacket().
// =========================================================================
void sendOnSignalPacket() {
  sendByteAsHex(0x11);  // 1st marker byte for "ON" event
  sendByteAsHex(0x11);  // 2nd marker byte
  sendByteAsHex(0x11);  // 3rd marker byte
  sendByteAsHex(0x11);  // 4th marker byte
  sendByteAsHex(0x11);  // 5th marker byte
  sendByteAsHex(0x00);  // Footer byte, always 0x00
}

// =========================================================================
// ================= DEDICATED OFF-TRANSITION SIGNAL PACKET ===============
// Sent ONLY at the exact moment the output is switched OFF.
//   byte[0..4] = 0x12, 0x12, 0x12, 0x12, 0x12   (first 5 bytes)
//   byte[5]    = 0x00                            (last byte)
// The (now zeroed) Vref value is sent right after it via sendVrefPacket(),
// automatically — this happens once, even without pressing SAVE.
// =========================================================================
void sendOffSignalPacket() {
  sendByteAsHex(0x12);  // 1st marker byte for "OFF" event
  sendByteAsHex(0x12);  // 2nd marker byte
  sendByteAsHex(0x12);  // 3rd marker byte
  sendByteAsHex(0x12);  // 4th marker byte
  sendByteAsHex(0x12);  // 5th marker byte
  sendByteAsHex(0x00);  // Footer byte, always 0x00
}

// =========================================================================
// ================= VREF PACKET (HEADER ALWAYS 0x00) =====================
// packet[0] : 0x00  <-- ALWAYS, every single time, no exceptions
// packet[1..4]: IEEE-754 float32 of vRef, Little-Endian, exact value
// packet[5] : 0x00 fixed footer
// Sent individually, one byte per line, as "0xXX".
// =========================================================================
void sendVrefPacket() {
  uint8_t dataBytes[4];                  // Will hold the 4 raw bytes of the vRef float
  floatToLE4Bytes(vRef, dataBytes);      // Convert vRef into its 4-byte representation

  sendByteAsHex(0x00);         // packet[0] - header, ALWAYS 0x00 for vRef
  sendByteAsHex(dataBytes[0]); // packet[1] - LSB
  sendByteAsHex(dataBytes[1]); // packet[2]
  sendByteAsHex(dataBytes[2]); // packet[3]
  sendByteAsHex(dataBytes[3]); // packet[4] - MSB
  sendByteAsHex(0x00);         // packet[5] - footer, always 0x00
}

// ================= SETUP =================
// setup() runs once when the ESP32 powers on or resets
void setup() {
  // Primary Native Serial Interface (USB Debug)
  Serial.begin(115200);   // Start USB serial at 115200 baud for debug prints
  
  // Secondary Hardware SCI Port (Serial2) mapped for Putty Terminal
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);  // Start UART2 (8 data bits, no parity, 1 stop bit)
  
  // Configure each button pin as an input with an internal pull-up resistor
  // (so the pin reads HIGH normally, and LOW only when the button is pressed)
  pinMode(BTN_ONOFF,   INPUT_PULLUP);
  pinMode(BTN_WAVE,    INPUT_PULLUP);
  pinMode(BTN_INC,     INPUT_PULLUP);
  pinMode(BTN_DEC,     INPUT_PULLUP);
  pinMode(BTN_SAVE,    INPUT_PULLUP);
  pinMode(BTN_CURSOR,  INPUT_PULLUP);

  randomSeed(analogRead(34));   // Seed the random number generator using noise from pin 34

  SPI.begin(18, 19, 23, TFT_CS);  // Start the SPI bus with clock, MISO, MOSI, and chip-select pins

  // Manually pulse the display's reset pin to make sure it boots cleanly
  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, LOW);   // Pull reset low to reset the display
  delay(50);                     // Hold it low briefly
  digitalWrite(TFT_RST, HIGH);  // Release reset
  delay(100);                    // Give the display time to finish booting

  tft.init(240, 320);      // Initialize the display driver with its resolution
  tft.setRotation(2);      // Portrait Mode (rotate display to the correct orientation)

  updateBuckParameters();  // Calculate initial voltage/current/power values
  clearEntireScreen();     // Wipe the screen before drawing
  drawDashboard();         // Draw the main dashboard UI
  
  // System starts OFF: send the OFF signal packet + zeroed Vref packet
  // once at boot, matching the same behavior as pressing OFF.
  sendOffSignalPacket();   // Tell the C2000 "output is OFF" at startup
  sendVrefPacket();        // Send the (zeroed) Vref value at startup

  setupEthernetLink();     // Start the UART link to the USR-TCP232 Ethernet bridge
}

// ================= LOOP =================
// loop() runs over and over forever after setup() finishes
void loop() {
  checkButtons();      // Check all 6 buttons for new presses every loop cycle
  checkPuttyInput();   // Check for any text typed in PuTTY and update the terminal buffer
  checkC2000Input();   // Check for any incoming packets from the C2000 board
  checkEthernetInput(); // Check for any commands arriving from the Python GUI over Ethernet

  // Push live data to the Python GUI at a fast, steady rate (50 times/sec)
  static unsigned long lastEthBroadcast = 0;
  if (millis() - lastEthBroadcast > 8) {
    broadcastEthernetData();
    lastEthBroadcast = millis();
  }

  // If the "SET!" badge has been showing for more than 1 second, hide it again
  if (showSetNotification && (millis() - setNotificationTimer > 1000)) {
    showSetNotification = false;
    if (currentPage == PAGE_DASHBOARD) {
      drawStatusArea();   // Redraw status area without the badge
    }
  }

  static unsigned long lastUpdate = 0;   // Remembers the last time we updated values (persists between loop calls)
  if (millis() - lastUpdate > 100) {     // Only run this block every 100 milliseconds
    inputVoltage = 23.5 + (random(0, 30) / 100.0);   // Simulate small random drift in input voltage
    updateBuckParameters();                           // Recalculate all buck converter values
    
    if (outputActive) {
      outputVoltage += (random(0, 20) / 1000.0) - 0.01;  // Add tiny random jitter to output voltage
      if (outputVoltage < 0) outputVoltage = 0;            // Never let voltage go negative
    }

    if (currentPage == PAGE_DASHBOARD) {
      updateValues();   // Refresh the on-screen numbers if we're viewing the dashboard
    }
    
    // NOTE: no serial transmission happens here. The periodic tick only
    // updates internal simulation values and the on-screen display —
    // it does NOT send anything to PuTTY. Vref packets are sent only
    // via SAVE, ON, or OFF (see handleButtonPress).
    
    lastUpdate = millis();   // Reset the timer for the next 800ms cycle
  }
}

// ================= BUTTON CHECK =================
// Reads all button pins and detects clean, debounced button presses
void checkButtons() {
  for (int i = 0; i < 6; i++) {                 // Loop through each of the 6 buttons
    int reading = digitalRead(buttons[i].pin);  // Read the current raw pin state

    if (reading != buttons[i].lastState) {      // If the reading changed since last time...
      buttons[i].debounceTime = millis();       // ...restart the debounce timer
    }

    if ((millis() - buttons[i].debounceTime) > 40) {   // If the state has been stable for 40ms (no bounce)
      if (reading == LOW && !buttons[i].pressed) {     // LOW means pressed (active-LOW), and not already registered
        handleButtonPress(buttons[i].pin);             // Trigger the action for this button
        buttons[i].pressed = true;                      // Mark it as pressed so we don't repeat-fire
      } else if (reading == HIGH) {
        buttons[i].pressed = false;                      // Button released, ready to detect next press
      }
    }
    buttons[i].lastState = reading;   // Remember this reading for comparison next loop
  }
}

// ================= DASHBOARD (MAIN) =================
// Draws the entire main dashboard screen from scratch
void drawDashboard() {
  clearEntireScreen();   // Wipe screen first
  drawOuterBorder();     // Draw the border frame

  tft.drawFastHLine(10, 82, SCREEN_W - 20, COLOR_GRID);   // Horizontal divider line under Voltage section
  tft.drawFastHLine(10, 162, SCREEN_W - 20, COLOR_GRID);  // Horizontal divider line under Current section

  tft.setTextSize(2);           // Medium-small text for labels
  tft.setTextColor(COLOR_LABEL);
  tft.setCursor(20, 15);   tft.print("Voltage");   // Section label
  tft.setCursor(20, 95);   tft.print("Current");   // Section label
  tft.setCursor(20, 175);  tft.print("Power");     // Section label

  updateValues();     // Draw the actual voltage/current/power numbers
  drawStatusArea();   // Draw the ON/OFF status badge area
  drawVref();          // Draw the Vref setting bar at the bottom
}

// ================= VALUE UPDATES =================
// Redraws just the numeric readouts (voltage/current/power) without redrawing everything else
void updateValues() {
  tft.setTextSize(4);   // Large text size for the big numbers

  // 1. Output Voltage Display
  tft.setCursor(20, 42);
  if (outputActive) {
    tft.setTextColor(COLOR_VOLTAGE, COLOR_BG);          // Cyan text on black background
    tft.print(String(outputVoltage, 2) + " V ");        // Print voltage with 2 decimal places
  } else {
    tft.setTextColor(COLOR_BORDER, COLOR_BG);           // Dimmed grey text when off
    tft.print("0.00 V ");
  }

  // 2. Output Current Display
  tft.setCursor(20, 122);
  if (outputActive) {
    tft.setTextColor(COLOR_CURRENT, COLOR_BG); 
    tft.print(String(outputCurrent, 2) + " A ");        // Print current with 2 decimal places
  } else {
    tft.setTextColor(COLOR_BORDER, COLOR_BG);
    tft.print("0.00 A ");
  }

  // 3. Power Display
  tft.setCursor(20, 202);
  if (outputActive) {
    tft.setTextColor(COLOR_POWER, COLOR_BG);
    
    char rawBuf[10];                          // Temporary buffer to format the power value
    dtostrf(outputPower, 6, 1, rawBuf);        // Convert float to string, width 6, 1 decimal place
    
    String pStr = String(rawBuf);              // Wrap it in an Arduino String object
    pStr.trim();                                // Remove any extra spaces
    while(pStr.indexOf('.') < 4) {              // Pad with leading zeros until 4 digits before the decimal
       pStr = "0" + pStr;
    }
    tft.print(pStr + "W");                      // Print the final formatted power string
  } else {
    tft.setTextColor(COLOR_BORDER, COLOR_BG);
    tft.print("0000.0W");
  }
}

// ================= HIGHLIGHTED STATUS BAR AREA =================
// Draws the ON/OFF pill button and the "SET!" badge
void drawStatusArea() {
  int statusY = SCREEN_H - 85;   // Y position of this status row, near the bottom
  
  tft.fillRect(10, statusY, SCREEN_W - 20, 32, COLOR_BG);   // Clear this area before redrawing

  if (outputActive) {
    tft.fillRoundRect(20, statusY, 90, 30, 4, HIGHLIGHT_ON);   // Filled magenta pill for ON
    tft.drawRoundRect(20, statusY, 90, 30, 4, COLOR_BORDER);   // Border outline around pill
    tft.setTextColor(COLOR_BG); 
    tft.setTextSize(2);
    tft.setCursor(51, statusY + 7);
    tft.print("ON");
  } else {
    tft.fillRoundRect(20, statusY, 90, 30, 4, HIGHLIGHT_OFF);  // Filled cyan pill for OFF
    tft.drawRoundRect(20, statusY, 90, 30, 4, COLOR_BORDER);
    tft.setTextColor(COLOR_BG); 
    tft.setTextSize(2);
    tft.setCursor(46, statusY + 7);
    tft.print("OFF");
  }

  if (showSetNotification) {
    tft.fillRoundRect(124, statusY, 95, 30, 4, COLOR_SET_BADGE);   // Pink "SET!" badge
    tft.drawRoundRect(124, statusY, 95, 30, 4, COLOR_TEXT);
    tft.setTextColor(COLOR_BG);
    tft.setTextSize(2);
    tft.setCursor(152, statusY + 7);
    tft.print("SET!");
  }
}

// ================= BOTTOM VREF SETTING BAR =================
// Draws the Vref digit-entry bar at the bottom of the dashboard, with a moving cursor highlight
void drawVref() {
  int footerY = SCREEN_H - 45;   // Y position of the footer bar
  
  tft.fillRect(4, footerY, SCREEN_W - 8, 41, COLOR_HEADER);   // Background box for the footer
  tft.drawFastHLine(4, footerY, SCREEN_W - 8, COLOR_BORDER);  // Top border line of the footer

  tft.setTextSize(2);
  tft.setTextColor(COLOR_LABEL);
  tft.setCursor(20, footerY + 12);
  tft.print("Vref:");   // Label text

  int startX = 90;   // X position where the digits start

  tft.setTextColor(COLOR_TEXT);
  tft.setCursor(startX,      footerY + 12); tft.print(vRef0);   // Print tens digit
  tft.setCursor(startX + 16, footerY + 12); tft.print(vRef1);   // Print ones digit
  tft.setCursor(startX + 31, footerY + 12); tft.print(".");     // Print decimal point
  tft.setCursor(startX + 38, footerY + 12); tft.print(vRef2);   // Print first decimal digit
  tft.setCursor(startX + 54, footerY + 12); tft.print(vRef3);   // Print second decimal digit
  tft.setCursor(startX + 72, footerY + 12); tft.print("V");     // Print units label

  if (currentPage == PAGE_DASHBOARD) {   // Only show the editing cursor on the dashboard page
    int cursorX = startX + (cursorIndex * 16);   // Calculate cursor's X position based on selected digit
    if (cursorIndex > 1) cursorX += 6;             // Extra offset to skip over the decimal point

    tft.fillRect(cursorX - 3, footerY + 6, 20, 26, COLOR_BG);            // Clear old cursor highlight area
    tft.fillRoundRect(cursorX - 3, footerY + 6, 20, 26, 4, COLOR_CURSOR); // Draw new yellow cursor highlight

    tft.setTextColor(COLOR_BG);
    tft.setCursor(cursorX, footerY + 12);

    int currentDigit = 0;   // Figure out which digit value is currently under the cursor
    if      (cursorIndex == 0) currentDigit = vRef0;
    else if (cursorIndex == 1) currentDigit = vRef1;
    else if (cursorIndex == 2) currentDigit = vRef2;
    else if (cursorIndex == 3) currentDigit = vRef3;

    tft.print(currentDigit);   // Re-print that digit on top of the highlight so it's still visible
  }
}

// ================= GRAPH AXES =================
// Draws simple X and Y axis lines for the waveform pages
void drawGraphAxes() {
  tft.drawFastVLine(30, 55, 160, COLOR_AXIS);    // Vertical Y-axis line
  tft.drawFastHLine(30, 215, 195, COLOR_AXIS);   // Horizontal X-axis line
}

// ================= VOLTAGE PAGE =================
// Draws a fake square-wave graphic to represent the voltage waveform
void drawVoltagePage() {
  clearEntireScreen();
  drawOuterBorder();
  tft.fillRect(4, 4, SCREEN_W - 8, 35, COLOR_HEADER);      // Header bar background
  tft.drawFastHLine(4, 39, SCREEN_W - 8, COLOR_BORDER);    // Line under header

  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT);
  tft.setCursor(20, 13);
  tft.print("VOLTAGE WAVE");   // Page title

  tft.drawRoundRect(10, 48, SCREEN_W - 20, 180, 6, COLOR_BORDER);   // Box frame around the graph
  drawGraphAxes();   // Draw X/Y axis lines

  int yHigh = 95, yLow = 175, startX = 35, cycleWidth = 40;   // Layout constants for the square wave
  for (int i = 0; i < 4; i++) {              // Draw 4 repeating square-wave cycles
    int x1 = startX + (i * cycleWidth);      // Start X of this cycle
    int x2 = x1 + (cycleWidth / 2);          // Midpoint X of this cycle
    if (x2 < SCREEN_W - 25) {                 // Only draw if it still fits on screen
      tft.drawFastHLine(x1, yHigh, cycleWidth / 2, COLOR_VOLTAGE);              // Top flat segment
      tft.drawFastVLine(x2, yHigh, yLow - yHigh,  COLOR_VOLTAGE);               // Falling edge
      tft.drawFastHLine(x2, yLow,  cycleWidth / 2, COLOR_VOLTAGE);              // Bottom flat segment
      if (i < 3) tft.drawFastVLine(x1 + cycleWidth, yHigh, yLow - yHigh, COLOR_VOLTAGE);  // Rising edge
    }
  }
  drawVref();   // Still show the Vref bar at the bottom on this page too
}

// ================= CURRENT PAGE =================
// Draws a fake triangle-wave graphic to represent the current waveform
void drawCurrentPage() {
  clearEntireScreen();
  drawOuterBorder();
  tft.fillRect(4, 4, SCREEN_W - 8, 35, COLOR_HEADER);
  tft.drawFastHLine(4, 39, SCREEN_W - 8, COLOR_BORDER);

  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT);
  tft.setCursor(20, 13);
  tft.print("CURRENT WAVE");   // Page title

  tft.drawRoundRect(10, 48, SCREEN_W - 20, 180, 6, COLOR_BORDER);
  drawGraphAxes();

  int yHigh = 95, yLow = 175, startX = 35, cycleWidth = 40;   // Layout constants for the triangle wave
  for (int i = 0; i < 4; i++) {                // Draw 4 repeating triangle-wave cycles
    int xStart = startX + (i * cycleWidth);    // Start X of this cycle
    int xMid   = xStart + (cycleWidth / 2);    // Midpoint (peak) X of this cycle
    int xEnd   = xStart + cycleWidth;           // End X of this cycle
    if (xEnd < SCREEN_W - 20) {                  // Only draw if it fits on screen
      tft.drawLine(xStart, yLow, xMid, yHigh, COLOR_CURRENT);   // Rising diagonal line
      tft.drawLine(xMid, yHigh, xEnd, yLow,   COLOR_CURRENT);   // Falling diagonal line
    }
  }
  drawVref();
}

// ================= POWER PAGE =================
// Draws a smooth sine-wave graphic to represent the power waveform
void drawPowerPage() {
  clearEntireScreen();
  drawOuterBorder();
  tft.fillRect(4, 4, SCREEN_W - 8, 35, COLOR_HEADER);
  tft.drawFastHLine(4, 39, SCREEN_W - 8, COLOR_BORDER);

  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT);
  tft.setCursor(20, 13);
  tft.print("POWER WAVE");   // Page title

  tft.drawRoundRect(10, 48, SCREEN_W - 20, 180, 6, COLOR_BORDER);
  drawGraphAxes();

  int centerY = 135, startX = 35;                   // Vertical center line and starting X of the wave
  int prevY = centerY - (int)(sin(0) * 35);         // Y position of the very first point on the wave

  for (int x = startX + 1; x <= SCREEN_W - 25; x++) {           // Step across the graph pixel by pixel
    int y = centerY - (int)(sin((x - startX) * 0.12) * 35);     // Calculate this point's Y using sine wave math
    tft.drawLine(x - 1, prevY, x, y, COLOR_POWER);                // Connect previous point to this point
    prevY = y;                                                     // Remember this Y for the next segment
  }
  drawVref();
}

// ================= PUTTY TERMINAL PAGE =================
// Draws the header + all currently buffered lines of text received from PuTTY
void drawTerminalPage() {
  clearEntireScreen();
  drawOuterBorder();
  tft.fillRect(4, 4, SCREEN_W - 8, 35, COLOR_HEADER);
  tft.drawFastHLine(4, 39, SCREEN_W - 8, COLOR_BORDER);

  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT);
  tft.setCursor(20, 13);
  tft.print("PUTTY TERMINAL");   // Page title

  tft.setTextSize(1);                           // Small text so more lines fit on screen
  tft.setTextColor(COLOR_OUTPUT, COLOR_BG);     // Green text on black, matches waveform color scheme

  int y = 48;                                    // Starting Y position for the first line of text
  for (int i = 0; i < termLineCount; i++) {
    tft.setCursor(10, y);
    tft.print(termLines[i]);
    y += 12;                                      // Move down for the next line
  }
}

// Adds one completed line of text to the scrolling buffer, shifting older lines up if full
void addTerminalLine(String line) {
  if (termLineCount < TERM_MAX_LINES) {
    // Buffer isn't full yet, just append the new line at the next free slot
    termLines[termLineCount] = line;
    termLineCount++;
  } else {
    // Buffer is full: shift every line up by one position, dropping the oldest
    for (int i = 0; i < TERM_MAX_LINES - 1; i++) {
      termLines[i] = termLines[i + 1];
    }
    termLines[TERM_MAX_LINES - 1] = line;   // Newest line goes in the last slot
  }

  if (currentPage == PAGE_TERMINAL) {
    drawTerminalPage();   // Only redraw immediately if the user is actually viewing this page
  }
}

// Reads whatever characters PuTTY has sent over the main USB Serial port,
// builds up lines, and pushes each completed line (ended by Enter) into
// the terminal buffer. Using Serial (not Serial2) means PuTTY connects
// to the SAME USB cable/COM port you already use to upload code — no
// extra USB-to-TTL adapter needed.
void checkPuttyInput() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      // Line finished (Enter was pressed) — push it to the display buffer.
      // We accept EITHER \r or \n as the terminator, since different
      // terminal programs (PuTTY vs Arduino Serial Monitor) send
      // different combinations. To avoid pushing a blank second line
      // when a program sends "\r\n" together, only push if there's
      // actually something in the buffer.
      if (termInputBuffer.length() > 0) {
        addTerminalLine(termInputBuffer);
        termInputBuffer = "";
      }
    } else if (c == 8 || c == 127) {
      // Handle backspace/delete (in case PuTTY sends it instead of doing local editing)
      if (termInputBuffer.length() > 0) {
        termInputBuffer.remove(termInputBuffer.length() - 1);
      }
    } else {
      termInputBuffer += c;                          // Append the typed character
      if (termInputBuffer.length() >= TERM_MAX_CHARS) {
        // Line got too long for one row on screen — wrap it automatically
        addTerminalLine(termInputBuffer);
        termInputBuffer = "";
      }
    }
  }
}

// ================= RECEIVE DATA FROM C2000 =================
// Reads hex-text lines like "0xAB" arriving on Serial2 (from the C2000
// board), reconstructs them into a 6-byte packet, and hands the
// completed packet off to handleC2000Packet() once all 6 bytes arrive.
void checkC2000Input() {
  while (Serial2.available()) {
    char c = Serial2.read();

    if (c == '\n' || c == '\r') {
      if (c2000RxLineBuffer.length() > 0) {
        // Convert the "0xXX" text into an actual byte value
        uint8_t byteVal = (uint8_t) strtol(c2000RxLineBuffer.c_str(), NULL, 16);
        c2000RxPacket[c2000RxByteIndex] = byteVal;
        c2000RxByteIndex++;

        if (c2000RxByteIndex >= 6) {           // Full 6-byte packet received
          handleC2000Packet(c2000RxPacket);
          c2000RxByteIndex = 0;                 // Reset for the next packet
        }
      }
      c2000RxLineBuffer = "";
    } else {
      c2000RxLineBuffer += c;
    }
  }
}

// Interprets one completed 6-byte packet from the C2000 and stores the
// decoded float value into the right variable based on the header byte:
//   header == 1  ->  Voltage value
//   header == 2  ->  Current value
//   header == 3  ->  Power value
// Packet layout: [header byte][4 float bytes, little-endian][0x00 footer]
void handleC2000Packet(uint8_t *pkt) {
  if (pkt[5] != 0x00) return;   // Reject anything that doesn't end with the expected footer

  union {
    float f;
    uint8_t b[4];
  } conv;
  conv.b[0] = pkt[1];
  conv.b[1] = pkt[2];
  conv.b[2] = pkt[3];
  conv.b[3] = pkt[4];

  if (pkt[0] == 1) {
    c2000Voltage = conv.f;
    c2000VoltageReceived = true;
  } else if (pkt[0] == 2) {
    c2000Current = conv.f;
    c2000CurrentReceived = true;
  } else if (pkt[0] == 3) {
    c2000Power = conv.f;
    c2000PowerReceived = true;
  } else {
    return;   // Unknown header byte — ignore the packet
  }

  c2000LastRxTime = millis();

  if (currentPage == PAGE_C2000) {
    drawC2000Page();   // Redraw immediately if the user is viewing this page
  }
}

// ================= C2000 DATA PAGE =================
// Shows the most recently received Voltage, Current, and Power values
// from the C2000 board, each updated independently as packets arrive.
void drawC2000Page() {
  clearEntireScreen();
  drawOuterBorder();
  tft.fillRect(4, 4, SCREEN_W - 8, 35, COLOR_HEADER);
  tft.drawFastHLine(4, 39, SCREEN_W - 8, COLOR_BORDER);

  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT);
  tft.setCursor(20, 13);
  tft.print("C2000 DATA");   // Page title

  tft.drawFastHLine(10, 90, SCREEN_W - 20, COLOR_GRID);
  tft.drawFastHLine(10, 160, SCREEN_W - 20, COLOR_GRID);

  // --- Voltage row ---
  tft.setTextSize(2);
  tft.setTextColor(COLOR_LABEL);
  tft.setCursor(20, 50);
  tft.print("Voltage");

  tft.setTextSize(3);
  tft.setCursor(20, 75);
  if (c2000VoltageReceived) {
    tft.setTextColor(COLOR_VOLTAGE, COLOR_BG);
    tft.print(String(c2000Voltage, 2) + " V ");
  } else {
    tft.setTextColor(COLOR_BORDER, COLOR_BG);
    tft.print("-- V ");
  }

  // --- Current row ---
  tft.setTextSize(2);
  tft.setTextColor(COLOR_LABEL);
  tft.setCursor(20, 120);
  tft.print("Current");

  tft.setTextSize(3);
  tft.setCursor(20, 145);
  if (c2000CurrentReceived) {
    tft.setTextColor(COLOR_CURRENT, COLOR_BG);
    tft.print(String(c2000Current, 2) + " A ");
  } else {
    tft.setTextColor(COLOR_BORDER, COLOR_BG);
    tft.print("-- A ");
  }

  // --- Power row ---
  tft.setTextSize(2);
  tft.setTextColor(COLOR_LABEL);
  tft.setCursor(20, 190);
  tft.print("Power");

  tft.setTextSize(3);
  tft.setCursor(20, 215);
  if (c2000PowerReceived) {
    tft.setTextColor(COLOR_POWER, COLOR_BG);
    tft.print(String(c2000Power, 2) + " W ");
  } else {
    tft.setTextColor(COLOR_BORDER, COLOR_BG);
    tft.print("-- W ");
  }

  // --- Last update time ---
  tft.setTextSize(1);
  tft.setTextColor(COLOR_LABEL, COLOR_BG);
  tft.setCursor(20, 260);
  if (c2000VoltageReceived || c2000CurrentReceived || c2000PowerReceived) {
    unsigned long secsAgo = (millis() - c2000LastRxTime) / 1000;
    tft.print("Last update: " + String(secsAgo) + "s ago");
  } else {
    tft.print("No data received yet");
  }
}

// =========================================================================
// ================= ETHERNET LINK SETUP ===================================
// Starts the hardware UART that talks to the USR-TCP232 module. The module
// itself must already be configured (see notes near the top of this file)
// to expose that UART as a TCP server on your LAN. No WiFi credentials,
// IP configuration, or socket code is needed on the ESP32 side at all -
// as far as this sketch is concerned it's just printing to a serial port.
// =========================================================================
void setupEthernetLink() {
  EthSerial.begin(ETH_BAUD, SERIAL_8N1, ETH_RXD, ETH_TXD);
  Serial.println("Ethernet bridge UART started (talking to USR-TCP232 module).");
  Serial.println("Make sure the module is configured as a TCP Server matching ETH_BAUD.");
}

// =========================================================================
// ================= READ COMMANDS ARRIVING FROM THE PYTHON GUI ===========
// Bytes sent by the PC over TCP show up here as plain UART bytes (the
// USR-TCP232 module converts them transparently). We buffer characters
// until we see '\n' (the Python side sends one command per line), then
// hand the completed line off to handleGuiCommand() - exactly the same
// command set/logic as the old WebSocket version.
// =========================================================================
void checkEthernetInput() {
  while (EthSerial.available()) {
    char c = EthSerial.read();
    if (c == '\n') {
      handleGuiCommand(ethRxLineBuffer);
      ethRxLineBuffer = "";
    } else if (c != '\r') {
      ethRxLineBuffer += c;
      if (ethRxLineBuffer.length() > 64) ethRxLineBuffer = "";  // Safety: drop runaway lines
    }
  }
}

// =========================================================================
// ================= HANDLE COMMANDS FROM THE PYTHON GUI ==================
// Recognizes simple text commands sent from the desktop app and performs
// the matching action, reusing your existing button-press logic where
// possible so behavior stays identical to pressing the real buttons.
// =========================================================================
void handleGuiCommand(String cmd) {
  cmd.trim();

  if (cmd == "TOGGLE_OUTPUT") {
    handleButtonPress(BTN_ONOFF);   // Same as pressing the physical ON/OFF button
  }
  else if (cmd == "NEXT_PAGE") {
    handleButtonPress(BTN_WAVE);    // Same as pressing the physical WAVE button
  }
  else if (cmd == "MOVE_CURSOR") {
    handleButtonPress(BTN_CURSOR);  // Same as pressing the physical CURSOR button
  }
  else if (cmd == "INC_DIGIT") {
    handleButtonPress(BTN_INC);     // Same as pressing the physical INC button
  }
  else if (cmd == "DEC_DIGIT") {
    handleButtonPress(BTN_DEC);     // Same as pressing the physical DEC button
  }
  else if (cmd == "SAVE") {
    handleButtonPress(BTN_SAVE);    // Same as pressing the physical SAVE button
  }
  else if (cmd.startsWith("SET_VREF:")) {
    // Expects e.g. "SET_VREF:12.34"
    float newVref = cmd.substring(9).toFloat();
    if (newVref < 0) newVref = 0;
    if (newVref > 99.99) newVref = 99.99;

    int whole = (int)newVref;                       // Integer part, e.g. 12
    int frac  = round((newVref - whole) * 100);      // Decimal part as 2 digits, e.g. 34

    vRef0 = whole / 10;
    vRef1 = whole % 10;
    vRef2 = frac / 10;
    vRef3 = frac % 10;

    updateBuckParameters();   // Recalculate everything with the new Vref
    sendVrefPacket();          // Send it out to the C2000 immediately, same as pressing SAVE

    if (currentPage == PAGE_DASHBOARD) {
      drawVref();
      updateValues();
    }
  }
}

// =========================================================================
// ================= BROADCAST LIVE DATA TO THE PYTHON GUI ================
// Sends a small JSON text packet, one per line, out over the Ethernet
// bridge UART. The USR-TCP232 module transparently forwards each byte to
// the connected TCP client (the Python GUI). Called on a timer from
// loop() — see below.
// =========================================================================
// Sends a small JSON text packet, one per line, out over the Ethernet
// bridge UART. The USR-TCP232 module transparently forwards each byte to
// the connected TCP client (the Python GUI). Called on a timer from
// loop() — see below.
void broadcastEthernetData() {
  const char* pageNames[] = {"DASHBOARD", "VOLTAGE", "CURRENT", "POWER", "TERMINAL", "C2000"};

  String json = "{";
  json += "\"outputActive\":" + String(outputActive ? "true" : "false") + ",";
  json += "\"voltage\":"  + String(outputVoltage, 2) + ",";
  json += "\"current\":"  + String(outputCurrent, 2) + ",";
  json += "\"power\":"    + String(outputPower, 2)   + ",";
  json += "\"vref\":"     + String(vRef, 2)           + ",";
  json += "\"c2000Voltage\":" + String(c2000Voltage, 2) + ",";
  json += "\"c2000Current\":" + String(c2000Current, 2) + ",";
  json += "\"c2000Power\":"   + String(c2000Power, 2)   + ",";
  json += "\"page\":\""       + String(pageNames[currentPage]) + "\",";
  json += "\"cursorIndex\":"  + String(cursorIndex);
  json += "}";

  EthSerial.println(json);   // One JSON object per line, newline-terminated
}


// Decides what should happen when a specific button is pressed
void handleButtonPress(int pin) {
  if (pin == BTN_ONOFF) {                 // Special handling for the ON/OFF button
    outputActive = !outputActive;         // Flip the output state (true<->false)
    currentPage  = PAGE_DASHBOARD;        // Always jump back to the dashboard when toggling
    
    if (outputActive) {
      updateBuckParameters();             // Recalculate values now that output is ON
    } else {
      // Reset everything to zero when turning output OFF
      outputVoltage=0; outputCurrent=0; outputPower=0; dutyCycle=0;
      vRef0=vRef1=vRef2=vRef3=0; vRef=0.0;
    }
    drawDashboard();   // Redraw the whole dashboard to reflect the new state

    if (outputActive) {
      // ON transition: dedicated 0x11 x5 + 0x00 signal packet,
      // then the current Vref value once, right after it.
      sendOnSignalPacket();   // Tell the C2000 board "output just turned ON"
      sendVrefPacket();        // Send the current target voltage
    } else {
      // OFF transition: dedicated 0x12 x5 + 0x00 signal packet,
      // then the now-zeroed Vref value once, right after it —
      // automatically, even though SAVE was not pressed.
      sendOffSignalPacket();  // Tell the C2000 board "output just turned OFF"
      sendVrefPacket();        // Send the now-zeroed target voltage
    }
    return;   // Done handling this button, skip the switch-statement below
  }

  switch (pin) {   // Handle all other buttons based on which pin was pressed
    case BTN_WAVE:
      currentPage = (ScreenPage)((currentPage + 1) % 6);   // Cycle through all 6 pages (wraps back to 0 after 5)
      switch (currentPage) {                                 // Redraw whichever page we just switched to
        case PAGE_DASHBOARD: drawDashboard();    break;
        case PAGE_VOLTAGE:   drawVoltagePage();  break;
        case PAGE_CURRENT:   drawCurrentPage();  break;
        case PAGE_POWER:     drawPowerPage();    break;
        case PAGE_TERMINAL:  drawTerminalPage(); break;
        case PAGE_C2000:     drawC2000Page();    break;
      }
      break;

    case BTN_CURSOR:
      if (currentPage == PAGE_DASHBOARD) {           // Cursor movement only matters on the dashboard
        cursorIndex = (cursorIndex + 1) % 4;          // Move to next digit, wrapping back to 0 after digit 3
        drawVref();                                     // Redraw the Vref bar to show the moved cursor
      }
      break;

    case BTN_INC:
      if (currentPage == PAGE_DASHBOARD) {
        // Increase whichever digit is currently selected by the cursor, wrapping 9 back to 0
        if      (cursorIndex == 0) vRef0 = (vRef0 + 1) % 10;
        else if (cursorIndex == 1) vRef1 = (vRef1 + 1) % 10;
        else if (cursorIndex == 2) vRef2 = (vRef2 + 1) % 10;
        else if (cursorIndex == 3) vRef3 = (vRef3 + 1) % 10;
        
        updateBuckParameters();   // Recalculate vRef and duty cycle with the new digit
        drawVref();                 // Redraw the Vref bar with updated digit
        updateValues();             // Redraw voltage/current/power numbers
        // NOTE: no serial output here — this is the "editing stage".
      }
      break;

    case BTN_DEC:
      if (currentPage == PAGE_DASHBOARD) {
        // Decrease whichever digit is currently selected by the cursor, wrapping 0 back to 9
        // (adding 9 and taking mod 10 is a trick to "subtract 1" while staying positive)
        if      (cursorIndex == 0) vRef0 = (vRef0 + 9) % 10;
        else if (cursorIndex == 1) vRef1 = (vRef1 + 9) % 10;
        else if (cursorIndex == 2) vRef2 = (vRef2 + 9) % 10;
        else if (cursorIndex == 3) vRef3 = (vRef3 + 9) % 10;
        
        updateBuckParameters();
        drawVref();
        updateValues();
        // NOTE: no serial output here — this is the "editing stage".
      }
      break;

    case BTN_SAVE:
      if (currentPage == PAGE_DASHBOARD) {
        // This is the ONLY place (besides ON/OFF transitions) where the
        // Vref packet gets sent during normal operation — the explicit
        // "set" action.
        sendVrefPacket();   // Actually transmit the currently-set Vref value
        
        // Debug logs alive on native hardware USB serial port
        Serial.print(F("[SAVE] Target Vref Saved successfully: "));
        Serial.print(vRef, 2);
        Serial.println(F(" V"));
        
        showSetNotification = true;        // Turn on the "SET!" badge
        setNotificationTimer = millis();   // Start the 1-second timer for how long it shows
        drawStatusArea();                    // Redraw status area to show the badge immediately
      }
      break;
  }
}
