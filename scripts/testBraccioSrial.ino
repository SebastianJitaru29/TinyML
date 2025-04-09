/*
  testBraccioSrial.ino

*/

#include <Servo.h>

// Define the servo objects that the Braccio library uses.
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

#include <Braccio.h>

void setup() {  
  Serial.begin(115200);
  Braccio.begin();
}

void loop() {
  // Check if a command has been received
  if (Serial.available()) {
    // Read until newline character
    String command = Serial.readStringUntil('\n');
    command.trim();  // Remove any extra whitespace

    // Parse the command (assumes the format "M1,M2,M3,M4,M5,M6")
    int angles[7];
    int index = 0;
    int lastIndex = 0;
    for (int i = 0; i < command.length() && index < 6; i++) {
      if (command.charAt(i) == ',') {
        angles[index++] = command.substring(lastIndex, i).toInt();
        lastIndex = i + 1;
      }
    }
    // Capture the final value after the last comma
    if (index < 7) {
      angles[index++] = command.substring(lastIndex).toInt();
    }

    // If we received exactly 6 numbers, perform the servo movement
    if (index == 7) {
      // The first parameter is the step delay (e.g., 20 ms)
      Braccio.ServoMovement(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6]);
      
      delay(1000); 

      // Optionally, send an acknowledgment back to the Python script:
      Serial.print("Command executed: ##");
      Serial.println(command);
    }
  }
}
