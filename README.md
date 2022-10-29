# Trash Categorizer

This project is the source code for the project I've been working on from September to August and 10/29 (deadline yo).
The model consists of a HuskyLens, an HCSR04, a Servo, an IR receiver and a 3-colored LED indicator. 
HuskyLens will be used to identify the type of object on a platform supported by the servo and the program will control the servo to tilt towards one of the two trash cans depending on the ID the HuskyLens read. EEPROM is used to hold which ID is which side respectively.