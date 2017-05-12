// -------------------------------------------------------------------------------------------
// I2C Bus Scanner
// https://github.com/nox771/i2c_t3/blob/master/examples/basic_scanner/basic_scanner.ino
// -------------------------------------------------------------------------------------------
//
// This creates an I2C master device which will scan the address space and report all
// devices which ACK.  It does not attempt to transfer data, it only reports which devices
// ACK their address.
//
// Pull the control pin low to initiate the scan.  Result will output to Serial.
//
// This example code is in the public domain.
// -------------------------------------------------------------------------------------------


#include <i2c_t3.h>

const int LED = LED_BUILTIN;
uint8_t target, found, all;
void print_scan_results(uint8_t target, uint8_t all);

void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
  pinMode(12,INPUT_PULLUP);       // pull pin 12 low to show ACK only results
  pinMode(11,INPUT_PULLUP);       // pull pin 11 low for a more verbose result (shows both ACK and NACK)
  

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.setDefaultTimeout(10000);
  
  Serial.begin(9600);
  Serial.println("Starting Program...");
  
}

void loop() {
  if(digitalRead(12) == LOW || digitalRead(11) == LOW) {
    all = (digitalRead(11) == LOW);
    found = 0;

    Serial.println("-------------------------------------\n");
    Serial.println("Starting scan...");

    digitalWrite(LED, HIGH);

    for(target = 1; target <= 0x7F; target++) {
      Wire.beginTransmission(target);
      Wire.endTransmission();
      print_scan_results(target, all);
      
    }

    digitalWrite(LED, LOW);

    if(!found) Serial.println("No devices found.\n");

    delay(100);
  }
}

void print_scan_results(uint8_t target, uint8_t all) {
  switch(Wire.status()){
    case I2C_WAITING:
      Serial.printf("Addr: 0x%02X Ack\n", target);
      found = 1;
      break;
    case I2C_ADDR_NAK:
      if (all) Serial.printf("Addr 0x%02X\n", target);
      break;
    default:
      break;
  }
}


