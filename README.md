# spider-quadruped
This code for a spider robot is based on kasinatorthh's code:
https://github.com/kasinatorhh/BTSpidey
The physical design is essentially the same as  kasinatorhh: https://www.thingiverse.com/thing:4070234 and instructions for assembly, calibration, etc can be found there. There are a few modifications to the non-moving parts. The ultrasonic ranging module is omitted, and there is a modified electronics holder to fit the Arduino Nano board, PCA9685, and charging circuit and connector for the LiPo battery. 

Following are the major modifications/enhancements in this code:
1. Runs on Arduino Nano 33 IoT rather than Ardunio Nano. The board includes an integrated bluetooth transceiver, so that a separate board is not needed to use a bluetooth gamepad to control the robot.
2. The Flexitimer2 library had to be replaced with the SAMDTimerInterrupt library.
3. Calibration values stored in flash rather than EEPROM (which is not available on this board)
4. Uses bluepad32 library and firmware for the gamepad controller: https://github.com/ricardoquesada/bluepad32/blob/main/docs/plat_nina.md
5. Servo calibration can be performed using the gamepad rather.
6. LiPo battery voltage monitor using analog pin A6 with a voltage divider: 82Kohm between Vin and A6 and 27kohm between A6 and Gnd.
7. Voltage level is displayed on the gamepad LEDs: 4 LEDs inidcating full charge and 1 LED low voltage threshhold.  The gamepad vibrates continuously when voltage drops below the low threshhold.
8. Added additional moves from John Crombie's code: https://www.youtube.com/watch?v=wmmPD2v2RAA
9. Added modules for polar/cartesian tranformation using cordic library for future port to ESP32, which does not allow floating point code in an interrupt service routine.

Any bluetooth gamepad supported by the bluepad32 library can be used. See the images folder for button actions using the gamepad controller.
