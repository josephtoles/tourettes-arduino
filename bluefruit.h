#include <Arduino.h>


// A small helper (Bluefruit)
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


void bluefruit_setup() {
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("*****************"));
}


/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
bool getUserInput(char buffer[], uint8_t maxSize)
{
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while( (Serial.peek() < 0) && !timeout.expired() ) {}

  if ( timeout.expired() ) return false;

  delay(2);
  uint8_t count=0;
  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && !(Serial.peek() < 0) );

  return true;
}


void bluefruit_loop() {

  //  Send arduino --> phone
  /*
  // Check for user input
  char inputs[BUFSIZE+1];
  if ( getUserInput(inputs, BUFSIZE) )
  {
    // Send characters to Bluefruit
    Serial.print("[Send] ");
    Serial.println(inputs);

    ble.print("AT+BLEUARTTX=");
    ble.println(inputs);

    // check response stastus
    if (! ble.waitForOK() ) {
      Serial.println(F("Failed to send?"));
    }
  }
  */

  int x = analogRead(1);
  Serial.print("[Send] ");Serial.println(x);
  ble.print("AT+BLEUARTTX=");ble.print(x);ble.println(" ");
  delay(50);
  if (! ble.waitForOK() ) {
     Serial.println(F("Failed to send?"));
  }

  /*
  int y = analogRead(2);
  ble.print("AT+BLEUARTTX=");ble.print("y=");ble.print(y);ble.println(" ");
  delay(50);
  if (! ble.waitForOK() ) {
     Serial.println(F("Failed to send?"));
  }

  int z = analogRead(3);
  ble.print("AT+BLEUARTTX=");ble.print("z=");ble.print(z);ble.println(" ");  
  delay(50);
  if (! ble.waitForOK() ) {
     Serial.println(F("Failed to send?"));
  }
  */

  //ble.println("generic text");

  /* Send info phone --> arduino
  // Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }
  // Some data was found, its in the buffer
  Serial.print(F("[Recv] ")); Serial.println(ble.buffer);
  ble.waitForOK();
  */
}

