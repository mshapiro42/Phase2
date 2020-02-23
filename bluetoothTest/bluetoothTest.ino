
// SoftwareSerial is used to communicate with the BT
#include <SoftwareSerial.h>

SoftwareSerial BT(4, 5); // Arduino RX, TX (BT Dout, Din)

void setup()
{
  // Initialize BT Software Serial port. Make sure the baud
  // rate matches your BT setting (9600 is default).
  BT.begin(9600); 
  printMenu(); // Print a helpful menu:

}

void loop()
{
  // In loop() we continously check to see if a command has been
  //  received.
  if (BT.available())
  {
    char c = BT.read();
    switch (c)
    {
    case 'w':      // If received 'w'
    case 'W':      // or 'W'
      writeAPin(); // Write analog pin
      break;
    case 'd':      // If received 'd'
    case 'D':      // or 'D'
      writeDPin(); // Write digital pin
      break;
    case 'r':      // If received 'r'
    case 'R':      // or 'R'
      readDPin();  // Read digital pin
      break;
    case 'a':      // If received 'a'
    case 'A':      // or 'A'
      readAPin();  // Read analog pin
      break;
    }
  }
}

// Write Digital Pin
// Send a 'd' or 'D' to enter.
// Then send a pin #
//   Use numbers for 0-9, and hex (a, b, c, or d) for 10-13
// Then send a value for high or low
//   Use h, H, or 1 for HIGH. Use l, L, or 0 for LOW
void writeDPin()
{
  while (BT.available() < 2)
    ; // Wait for pin and value to become available
  char pin = BT.read();
  char hl = ASCIItoHL(BT.read());

  // Print a message to let the control know of our intentions:
  BT.print("Setting pin ");
  BT.print(pin);
  BT.print(" to ");
  BT.println(hl ? "HIGH" : "LOW");

  pin = ASCIItoInt(pin); // Convert ASCCI to a 0-13 value
  pinMode(pin, OUTPUT); // Set pin as an OUTPUT
  digitalWrite(pin, hl); // Write pin accordingly
}

// Write Analog Pin
// Send 'w' or 'W' to enter
// Then send a pin #
//   Use numbers for 0-9, and hex (a, b, c, or d) for 10-13
//   (it's not smart enough (but it could be) to error on
//    a non-analog output pin)
// Then send a 3-digit analog value.
//   Must send all 3 digits, so use leading zeros if necessary.
void writeAPin()
{
  while (BT.available() < 4)
    ; // Wait for pin and three value numbers to be received
  char pin = BT.read(); // Read in the pin number
  int value = ASCIItoInt(BT.read()) * 100; // Convert next three
  value += ASCIItoInt(BT.read()) * 10;     // chars to a 3-digit
  value += ASCIItoInt(BT.read());          // number.
  value = constrain(value, 0, 255); // Constrain that number.

  // Print a message to let the control know of our intentions:
  BT.print("Setting pin ");
  BT.print(pin);
  BT.print(" to ");
  BT.println(value);

  pin = ASCIItoInt(pin); // Convert ASCCI to a 0-13 value
  pinMode(pin, OUTPUT); // Set pin as an OUTPUT
  analogWrite(pin, value); // Write pin accordingly
}

// Read Digital Pin
// Send 'r' or 'R' to enter
// Then send a digital pin # to be read
// The Arduino will print the digital reading of the pin to BT.
void readDPin()
{
  while (BT.available() < 1)
    ; // Wait for pin # to be available.
  char pin = BT.read(); // Read in the pin value

  // Print beggining of message
  BT.print("Pin ");
  BT.print(pin);

  pin = ASCIItoInt(pin); // Convert pin to 0-13 value
  pinMode(pin, INPUT); // Set as input
  // Print the rest of the message:
  BT.print(" = "); 
  BT.println(digitalRead(pin));
}

// Read Analog Pin
// Send 'a' or 'A' to enter
// Then send an analog pin # to be read.
// The Arduino will print the analog reading of the pin to BT.
void readAPin()
{
  while (BT.available() < 1)
    ; // Wait for pin # to be available
  char pin = BT.read(); // read in the pin value

  // Print beginning of message
  BT.print("Pin A");
  BT.print(pin);

  pin = ASCIItoInt(pin); // Convert pin to 0-6 value
  // Printthe rest of the message:
  BT.print(" = ");
  BT.println(analogRead(pin));
}

// ASCIItoHL
// Helper function to turn an ASCII value into either HIGH or LOW
int ASCIItoHL(char c)
{
  // If received 0, byte value 0, L, or l: return LOW
  // If received 1, byte value 1, H, or h: return HIGH
  if ((c == '0') || (c == 0) || (c == 'L') || (c == 'l'))
    return LOW;
  else if ((c == '1') || (c == 1) || (c == 'H') || (c == 'h'))
    return HIGH;
  else
    return -1;
}

// ASCIItoInt
// Helper function to turn an ASCII hex value into a 0-15 byte val
int ASCIItoInt(char c)
{
  if ((c >= '0') && (c <= '9'))
    return c - 0x30; // Minus 0x30
  else if ((c >= 'A') && (c <= 'F'))
    return c - 0x37; // Minus 0x41 plus 0x0A
  else if ((c >= 'a') && (c <= 'f'))
    return c - 0x57; // Minus 0x61 plus 0x0A
  else
    return -1;
}

// printMenu
// A big ol' string of Serial prints that print a usage menu over
// to the other BT.
void printMenu()
{
  // Everything is "F()"'d -- which stores the strings in flash.
  // That'll free up SRAM for more importanat stuff.
  BT.println();
  BT.println(F("Arduino BT Remote Control!"));
  BT.println(F("============================"));
  BT.println(F("Usage: "));
  BT.println(F("w#nnn - analog WRITE pin # to nnn"));
  BT.println(F("  e.g. w6088 - write pin 6 to 88"));
  BT.println(F("d#v   - digital WRITE pin # to v"));
  BT.println(F("  e.g. ddh - Write pin 13 High"));
  BT.println(F("r#    - digital READ digital pin #"));
  BT.println(F("  e.g. r3 - Digital read pin 3"));
  BT.println(F("a#    - analog READ analog pin #"));
  BT.println(F("  e.g. a0 - Read analog pin 0"));
  BT.println();
  BT.println(F("- Use hex values for pins 10-13"));
  BT.println(F("- Upper or lowercase works"));
  BT.println(F("- Use 0, l, or L to write LOW"));
  BT.println(F("- Use 1, h, or H to write HIGH"));
  BT.println(F("============================"));  
  BT.println();
}
