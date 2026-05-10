#include <SCServo.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
SCSCL st; // SC15 Protocol Object

int RX_PIN = 18;
int TX_PIN = 19;

void setup() {
  Serial.begin(230400);                        
  Serial1.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN); 

  st.pSerial = &Serial1;
  SerialBT.begin("Hexapod_BT"); 
  Serial.println("The device started, now you can pair it with bluetooth!");
  
  // Apply the ultra-fast timeout to the BLUETOOTH port, not the USB port
  SerialBT.setTimeout(2); 
  delay(1000); 
}

void loop() {
  // MUST listen to SerialBT!
  if (SerialBT.available() > 0) { 
    
    // Look for the start marker
    if (SerialBT.read() == '<') {
      
      // Parse from the Bluetooth port
      int count = SerialBT.parseInt(); 

      // Safety check: max 18 motors
      if (count > 0 && count <= 18) {
        
        u8 ids[18];
        u16 positions[18];
        u16 times[18];   
        u16 speeds[18];  

        // Parse the ID and Step pairs
        for (int i = 0; i < count; i++) {
          ids[i] = SerialBT.parseInt();
          positions[i] = SerialBT.parseInt();
          times[i] = 15;  // 15ms travel time
          speeds[i] = 0;  // 0 speed
        }

        // FIRE ALL MOTORS SIMULTANEOUSLY
        st.SyncWritePos(ids, count, positions, times, speeds);
      }
    }
  }
}
