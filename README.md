# BROBOT-EVO2-MEGA-2560

This project is a modification of the JJRobots EVO2 balancing robot:
https://github.com/jjrobots/B-ROBOT_EVO2

I wanted more I/O and program space - so replaced the Arduino Leonardo in the kit with an Arduino Mega 2560.

Since the Mega 2560 is over an inch longer - I used Meshmixer to cut the Stealth files and Simplify3d to stretch and merge the pieces to extend the width of the chassis to 130mm.  My efforts did not get both of the mounting holes in the rear bumper STL - I just drilled a hole manually. 

I had to modify the Leonardo program files for several things that are different in the Mega 2560:
- Mega i2c pins are outside the EVO2 shield - so I plugged two long header pins into the Mega i2c pins and plugged in the i2c CLK and DATA pins from the Gyro cable into the Mega.
- Mega Serial0 (console) is on pins 0 and 1, so I unsoldered the shield pins 0 and 1 out and moved the pins to the top of the board and jumpered the pins to Mega TX1 and RX1.
- I changed the servo code to use the SoftwareServo library. The Servo library conflicted with the Wire library. Then I changed the servo code to make library calls. If I was more proficient at Arduino timer code - I might have been able to not use the softwareservo library and initialize the Mega timers.
- I changed the PORT and bit from Leonardo to Mega 2560 to get the stepper motor code to work with the Mega.
- I merged my Touch OSC code changes which provide battery voltage and battery percentage plus PID values on the PID screen

This is still a work in progress:  the additional mass has not been factored into the default PIDs, so the Mega EVO2 is drivable in PRO mode, but not as stable as the Leonardo EVO2 yet.
