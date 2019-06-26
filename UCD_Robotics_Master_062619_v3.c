/* Line Scan Camera Test Program
 * 10/31/2018 : David Slaughter Updated for 2018, with the image inverted right to left because the camera in 2018
 *                              is rotated 180 degrees from the prior orientation.  Also updated to the Feedback 360
 *                              servomotors.  Enhanced the image printout and set the gain to an integer (1.0).
 * 10/16/2016 : David Slaughter (first code)
 * 02/01/2017 : Thuy Nguyen (revisions)
 * 03/03/2017 : TN (displayed camera images to LCD with sampling, commented codes for auto-exposure for fast speed)
 * 05/10/2017 : DCS changed the code to input a CMOS binary value rather than the continuous valued analog value
 *              because Parallax does not have a CLK synchronized analog to digital converter it takes 100 ms for
 *              analog to digal conversion of 128 pixels.
 *              For direct binary image input, image readout takes ~10 ms, plus image exposure time.
 *
 * Note:
 *   - The TSL1401-DB contains a TAOS TSL1401R 128-pixel grayscale line scan camera
 *     and a 7.9mm lens that provides a field of view equal to subject distance.
 *   - The circuit board of the TAOS TSL1401R line scan camera is translucent (backlight
 *     will pass through the board) thus, you need to enclose the back of the camera to
 *     prevent backlight from affecting the image.
 *      PIN_SI_B and PIN_CLK_C are high to turn on the clock and start an integrate scan.
 *      This code generally means 1 cycle to wait for exposure time, 1 cycle to capture the picture, then loop back.
 */

#include "simpletools.h"                      // Include simple tools
#include "abdrive360.h"
#include "adcDCpropab.h"                      // Include adcDCpropab library
#include "servo.h"


 // for robot
#define TRAVELSPEED 30
#define CAMERA_ORIENTATION -1     // Use -1 for camera connector to front, use +1 for connector to back
#define LEFT_WHEEL 12
#define LEFT_WHEEL_FEEDBACK 14
#define RIGHT_WHEEL 13
#define RIGHT_WHEEL_FEEDBACK 15
#define SERVO_OFFSET 0
#define KP_HEADING 1.0
#define KD_HEADING 1.0
#define KI_HEADING 0.0001

// for lights

// for line-scan camera
#define PIN_AO_A          10   // AO (analog pixel input from the sensor: 0 – Vdd, or tri-stated if beyond pixel 128)  // This is ANALOG pin
#define PIN_SI_B          9   // SI (digital output to the sensor: begins a scan/exposure)
#define PIN_CLK_C         8   // CLK (digital output to the sensor: latches SI & clocks the pixels out)   // A, B, and C are DB-Expander pins
#define OSCILLOSCOPE      0   // synchronization pulse for oscilloscope for focusing camera
#define IMAGE_EDGE        4
#define EXPOSURE_TIME_MS  10
#define LSBPRE 1
#define MSBPRE 0

#define LINE_SCAN_LENGTH        128
#define CLOCKS_UNTIL_EXPOSURE   18

// for IR sensor
#define IR_PIN_LEFT     0
#define IR_PIN_RIGHT    1

// for color sensor
#define CS_PIN_RED      2
#define CS_PIN_GREEN    3

// for claw servo
#define SERVO_PIN   16


static volatile short int zeroSum, lineLoc, setpoint = 63, posError[5], leftSpeed, rightSpeed, controlSignal;
static volatile int frequency, expTime_ms, act_exp_time, expSum, mj, speed = 1;
static volatile int binaryImage[4], derError[2], intError, IR_PLANT, IR_PIN, BOARD_DIRECTION, COLOR, MASTER_COLOR, IR_PLANT_DROP;
//static volatile unsigned char *b;
static unsigned int stack[40 + 34], stack2[40 + 34], stack3[40 + 34]; // stacks for cogs, add more memory if you add variables or code.

void printImageToTerminal() { // separate cog function for printing
	int t, i, image[8];
	t = CNT;
	//simpleterm_open();                        // Open SimpleIDE Terminal for this core
	print("open");
	while (1) {
		if (CNT > (t + CLKFREQ / 3)) {
			print("%cExposure time=%d microseconds%c\n\nFar Left___________________Left of Center______________________Center\n",
				CLS, act_exp_time, CLREOL);
			/* The following code is needed when the camera is oriented with the connector to the back.
			// Make a local copy of the binary image.  Change binaryImage to image below if this code is used.
			image[4] = binaryImage[0];
			image[5] = binaryImage[1];
			image[6] = binaryImage[2];
			image[7] = binaryImage[3];
			// swap the MSB to LSB order of the bits for a spatially correct display
			for(i=0, image[0]=0; i<32; i++)
			  image[0] |= ((image[4]>>i) & 0b1)<<(31-i);
			for(i=0, image[1]=0; i<32; i++)
			  image[1] |= ((image[5]>>i) & 0b1)<<(31-i);
			for(i=0, image[2]=0; i<32; i++)
			  image[2] |= ((image[6]>>i) & 0b1)<<(31-i);
			for(i=0, image[3]=0; i<32; i++)
			  image[3] |= ((image[7]>>i) & 0b1)<<(31-i);
			  */
			print("|%32b|", binaryImage[3]);       // This is the left most 32 pixels
			print("%32b|\n\n", binaryImage[2]);  // This is the left-center 32 pixels

			print("Center_____________________Right of Center__________________Far Right\n");
			print("|%32b|", binaryImage[1]);    // This is the right-center 32 pixels
			print("%32b|\n\n", binaryImage[0]);    // This is the right most 32 pixels
			print("Line Location = %u, Zero Sum = %d  ExpSum=%d, mj=%d\n", lineLoc, zeroSum, expSum, mj);
			print("Error=%d derError=%d sgError=%d\n", posError[4], derError[0], derError[1]);
			print("LeftSpeed=%d RightSpeed=%d\n", leftSpeed, rightSpeed);
			print("Integral error: %d \n", intError);
			t = CNT;
		} // end of if time check
	}// end of while loop
}

void bitSearch() { // separate cog to find center of the line
	int bitTest, bitSum[4];
	short int i, pixelLoc; // pixelLoc is the sum of the pixel locations with a pixel with a value of 1

	while (1) {
		while (input(OSCILLOSCOPE) == 0); // wait for new image to be available.  Oscilloscope goes low at end of image read cycle.
		bitTest = 0b1; // set test bit to 1
		for (i = 0, pixelLoc = 0, bitSum[0] = bitSum[1] = bitSum[2] = bitSum[3] = 0; i < 32; i++) { // this loop counts pixels with the value 1
			if (lineLoc < 44 && (binaryImage[0] & bitTest)) { bitSum[0]++; pixelLoc += i; } // only analyze the first 32 pixels if lineLoc < 44
			if (binaryImage[1] & bitTest) { bitSum[1]++; pixelLoc += i + 32; }
			if (binaryImage[2] & bitTest) { bitSum[2]++; pixelLoc += i + 64; }
			if (lineLoc > 84 && (binaryImage[3] & bitTest)) { bitSum[3]++; pixelLoc += i + 96; } // only analyze the last 32 pixels if lineLoc > 84
			bitTest = bitTest << 1; // shift test bit by one binary position and loop back through
		} // end of for loop
		zeroSum = 0 - bitSum[0] - bitSum[1] - bitSum[2] - bitSum[3];
		// for reference: lineLoc = (8128 - j)/zeroSum; // 127*128/2 = 8128 is the sum of all 128 positions
		// determine the line center by finding the sum of the pixel locations with a value of zero (by subtraction) and dividing by the total.
		if (lineLoc < 44) { zeroSum += 96; lineLoc = (4560 - pixelLoc) / zeroSum; }
		else if (lineLoc > 84) { zeroSum += 96; lineLoc = (7632 - pixelLoc) / zeroSum; }
		else { zeroSum += 64; lineLoc = (4064 - pixelLoc) / zeroSum; }
		// make a copy of the most recent 5 position errors, the oldest is in posError[0] and the most recent is in posError[4]
		for (i = 0; i < 4; i++) posError[i] = posError[i + 1];
		posError[4] = setpoint - lineLoc;
		derError[0] = 5000 * (posError[4] - posError[3]) / (speed*(expTime_ms + 10));  // calculate a two point derivative term
		derError[1] = 5000 * (2 * posError[4] + posError[3] - posError[1] - 2 * posError[0]) / (speed*(expTime_ms + 10));  // Quadratic S.G. derivative
		intError += posError[4]; // calculate the integral error term
	}
}

void getImage() { // separate cog to acquire a new image
	int t;
	t = CNT;

	low(OSCILLOSCOPE);   // set oscilloscope pin high. Turns the LED lights off.
	low(PIN_SI_B);       // SI pin low
	while ((CNT - t) >= EXPOSURE_TIME_MS) {
		// Expose new image
		/*     frequency = 130 * 1000 / expTime_ms ; // cannot exceed 128 MHz
			 high(PIN_SI_B);   // SI pin high
			 high(PIN_CLK_C);  // CLK pin high
			 low(PIN_SI_B);   // SI pin low
			 low(PIN_CLK_C);  // CLK pin low
			 t=CNT;
		// the freqout() function is used to quickly modulate the clock pin to allow very short exposure times in snapshot mode.
			 freqout(PIN_CLK_C,  expTime_ms, frequency);  // Modulate CLK pin for ~130 cycles (extra above 128 is OK)
			 high(PIN_SI_B);   // SI pin high, ends exposure cycle
			 act_exp_time = CNT - t; // store actual exposure time
			 act_exp_time /= st_usTicks; // convert to microseconds
		*/
		// Transfer in the image data as binary, CMOS requires that the voltage be below 0.8 volts to be considered low.
		// Thus the exposure time used must allow the black like to read 0 in the binary image.
		high(OSCILLOSCOPE); // set oscilloscope pin high.  Used to synchronize oscilloscope and turn the lights off.
		low(PIN_SI_B);   // SI pin low
		high(PIN_SI_B);  // SI pin high
		high(PIN_CLK_C); // CLK pin high
		low(PIN_SI_B);   // SI pin low
   // Read image out of camera
		binaryImage[0] = shift_in(PIN_AO_A, PIN_CLK_C, LSBPRE, 32); // shift_in transfers a maximum of 32 bits
		binaryImage[1] = shift_in(PIN_AO_A, PIN_CLK_C, LSBPRE, 32); // MSB is on the left, LSB is on the right
		binaryImage[2] = shift_in(PIN_AO_A, PIN_CLK_C, LSBPRE, 32);
		binaryImage[3] = shift_in(PIN_AO_A, PIN_CLK_C, LSBPRE, 32);
		high(PIN_SI_B);   // SI pin high
		low(PIN_SI_B);   // SI pin low
		low(OSCILLOSCOPE); // set oscilloscope pin low.  Turns the lights on for the next exposure

	}
}

// -------------------------------------------------
// ----------- Master Code Fxns --------------------
// -------------------------------------------------

unsigned int countSetBits(unsigned int n) {
	unsigned int count = 0;
	while (n) {
		count += n & 1;
		n >>= 1;
	}
	return (128 - count); //128 = 32 + 32 + 32 + 32 (i.e. all of the binary image values)
}

void set_board_direction(void) {
	double IR_RIGHT_VOLTS; //right IR sensor
	double IR_LEFT_VOLTS; //left IR sensor
	int counter, SUM;

	IR_RIGHT_VOLTS = 0.0;
	IR_LEFT_VOLTS = 0.0;

  SUM = 0;
	counter = 50;

	while (counter > 0) { // take 10 different point and use the sum for which to use

		IR_RIGHT_VOLTS = adc_volts(IR_PIN_RIGHT);
		IR_LEFT_VOLTS = adc_volts(IR_PIN_LEFT);

     if ( IR_RIGHT_VOLTS > IR_LEFT_VOLTS ){
       SUM = SUM + 1;
     }
     else{
       SUM = SUM - 1;
     }

		counter = counter - 1;
		pause(55);
     print("\x10");
     print("Sum: %d", SUM);

	}


	if (SUM > 0) {
		BOARD_DIRECTION = 1; //right side of the board
	}
	else {
		BOARD_DIRECTION = -1; //left side of the board
	}

}

void go_straight_check_intersection(void) {

	unsigned int leftBits; //All of the bits read in by the line following camera from center-left -> left
	unsigned int rightBits; //All of the bits read in by the line following camera from center-right -> right

	leftBits = countSetBits(binaryImage[3]) + countSetBits(binaryImage[2]);
	rightBits = countSetBits(binaryImage[1]) + countSetBits(binaryImage[0]);

	//

	while (leftBits + rightBits < 410) {
		// navigation loop goes here
		controlSignal = (int)(KP_HEADING * ((float)posError[4]) + (intError*KI_HEADING) + (derError[0] * KD_HEADING)); // posError[4] contains the most recent position error

																														   // if integral or derivative control is desired
		if (controlSignal > 20) controlSignal = 20; // limit the control signal to avoid wild overcorrections
		if (controlSignal < -20) controlSignal = -20;

		if (abs(controlSignal) < 10) {
			leftSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (-1))*(controlSignal);  // 2018 Camera Orientation: Connector at Front
			rightSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (+1))* (controlSignal); // Adjust the left and right wheel speeds
		}
		else {
			leftSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (-1))*controlSignal;  // 2018 Camera Orientation: Connector at Front
			rightSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (+1))* controlSignal; // Adjust the left and right wheel speeds
		}



		/*leftSpeed = TRAVELSPEED + ( CAMERA_ORIENTATION * (-1))*controlSignal;  // 2018 Camera Orientation: Connector at Front
		rightSpeed = TRAVELSPEED + ( CAMERA_ORIENTATION * (+1))* controlSignal; // Adjust the left and right wheel speeds */
		drive_speed(leftSpeed, rightSpeed); // set the drive_speed based upon the control signal
		print(" %d , %d ", leftBits, rightBits);
		leftBits = countSetBits(binaryImage[3]) + countSetBits(binaryImage[2]);
		rightBits = countSetBits(binaryImage[1]) + countSetBits(binaryImage[0]);


	}

	leftSpeed = 0;
	rightSpeed = 0;
	drive_speed(leftSpeed, rightSpeed);


}

void pick_plant(void) {
	// Insert code here

	leftSpeed = TRAVELSPEED;
	rightSpeed = TRAVELSPEED;

	drive_speed(leftSpeed, rightSpeed);

	pause(800);

	rotate_robot_half_turn(BOARD_DIRECTION);

	leftSpeed = -1 * TRAVELSPEED;
	rightSpeed = -1 * TRAVELSPEED;

	drive_speed(leftSpeed, rightSpeed);

	pause(1300);

	leftSpeed = 0;
	rightSpeed = 0;

	drive_speed(leftSpeed, rightSpeed);

	servo_angle(SERVO_PIN, 0);                         // P16 servo to 0 degrees
	pause(1000);                                // ...for 3 seconds
	servo_angle(SERVO_PIN, 1800);                      // P16 servo to 180 degrees
	pause(1000);                                // ...for 3 seconds

	leftSpeed = TRAVELSPEED;
	rightSpeed = TRAVELSPEED;

	drive_speed(leftSpeed, rightSpeed);
	pause(1300);
	rotate_robot_half_turn(-1 * BOARD_DIRECTION);


}

void drop_plant(void) {
	// Insert code here

	leftSpeed = TRAVELSPEED;
	rightSpeed = TRAVELSPEED;

	drive_speed(leftSpeed, rightSpeed);

	pause(800);

	rotate_robot_half_turn(BOARD_DIRECTION);

	leftSpeed = -1 * TRAVELSPEED;
	rightSpeed = -1 * TRAVELSPEED;

	drive_speed(leftSpeed, rightSpeed);

	pause(800);

	leftSpeed = 0;
	rightSpeed = 0;

	drive_speed(leftSpeed, rightSpeed);

	servo_angle(SERVO_PIN, 0);                         // P16 servo to 0 degrees
	pause(1000);                                // ...for 3 seconds

	leftSpeed = TRAVELSPEED;
	rightSpeed = TRAVELSPEED;

	drive_speed(leftSpeed, rightSpeed);
	pause(800);
	rotate_robot_half_turn(-1 * BOARD_DIRECTION);


}

void rotate_robot(int Cwise_or_CCwise) {
	// Turns robot 90 deg to left or right
	// Input can either be a -1 (counterclockwise) or +1 (clockwise)

	int leftSpeed, rightSpeed;
	float axel_length; // Distance (mm) between wheels
	float ticks_per_mm; // Ticks per mm defined by abdrive.h
	float turn_speed; // Time (s) to complete the 90 degree turn

	axel_length = 140;
	ticks_per_mm = 3.25;
	turn_speed = 1.5;

	leftSpeed = (3.14 * (axel_length / 4.0) * (1.0 / turn_speed) * (1.0 / ticks_per_mm));
	rightSpeed = (3.14 * (axel_length / 4.0) * (1.0 / turn_speed) * (1.0 / ticks_per_mm));

	//print("\n %d %d", leftSpeed, rightSpeed);


	if (Cwise_or_CCwise > 0) {
		leftSpeed = leftSpeed;
		rightSpeed = rightSpeed * -0.5; // Slows down the inner wheel for a smoother turn
	}

	if (Cwise_or_CCwise < 0) {
		leftSpeed = leftSpeed * -0.5; // Slows down the inner wheel for a smoother turn
		rightSpeed = rightSpeed;
	}

	drive_speed(leftSpeed, rightSpeed);

	pause(turn_speed * 1250.0);  // Slightly increased the turning time because it wasnt rotating fully

	leftSpeed = 0;
	rightSpeed = 0;

	drive_speed(leftSpeed, rightSpeed);

}

void rotate_robot_half_turn(int Cwise_or_CCwise) {
	// Turns robot 90 deg to left or right
 	// Input can either be a -1 (counterclockwise) or +1 (clockwise)

 	int leftSpeed, rightSpeed;
 	float axel_length; // Distance (mm) between wheels
 	float ticks_per_mm; // Ticks per mm defined by abdrive.h
 	float turn_speed; // Time (s) to complete the 90 degree turn

 	axel_length = 140;
 	ticks_per_mm = 3.25;
 	turn_speed = 1.5;

 	leftSpeed = (3.14 * (axel_length / 4.0) * (1.0 / turn_speed) * (1.0 / ticks_per_mm));
 	rightSpeed = (3.14 * (axel_length / 4.0) * (1.0 / turn_speed) * (1.0 / ticks_per_mm));

 	//print("\n %d %d", leftSpeed, rightSpeed);


 	if (Cwise_or_CCwise > 0) {
 		leftSpeed = leftSpeed;
 		rightSpeed = rightSpeed * -0.5; // Slows down the inner wheel for a smoother turn
 	}

 	if (Cwise_or_CCwise < 0) {
 		leftSpeed = leftSpeed * -0.5; // Slows down the inner wheel for a smoother turn
 		rightSpeed = rightSpeed;
 	}

 	drive_speed(leftSpeed, rightSpeed);

 	pause(turn_speed * 850.0);  // Slightly increased the turning time because it wasnt rotating fully

 	leftSpeed = 0;
 	rightSpeed = 0;

 	drive_speed(leftSpeed, rightSpeed);
}

void check_IR_distance(void) {

	int counter;
	float IR_volts;

	counter = 10;
	IR_volts = 0.0;

	while (counter > 0) {
		IR_volts = adc_volts(IR_PIN) + IR_volts;
		counter = counter - 1;
	}

	IR_volts = IR_volts / 10.0;

	if (IR_volts > 0.4) {
		IR_PLANT = 1; //plant present
	}

	if (IR_volts < 0.4) {
		IR_PLANT = 0; //plant not present
	}
  print("%d \n", IR_PLANT);
}

void check_IR_drop(void) {

	int counter;
	float IR_volts;

	counter = 10;
	IR_volts = 0.0;

	while (counter > 0) {
		IR_volts = adc_volts(IR_PIN) + IR_volts;
		counter = counter - 1;
	}

	IR_volts = IR_volts / 10.0;

	if (IR_volts > 0.6) {
		IR_PLANT_PLANT = 1; //plant present
	}

	if (IR_volts < 0.6) {
		IR_PLANT_DROP = 0; //plant not present
	}
  print("%d \n", IR_PLANT);
}

void go_straight_check_IR(void) {
	// Insert code here
	// navigation loop goes here


	int delaycounter;

	delaycounter = 50;

	while (delaycounter > 0) {
		controlSignal = (int)(KP_HEADING * ((float)posError[4]) + (intError*KI_HEADING) + (derError[0] * KD_HEADING)); // posError[4] contains the most recent position error

																														   // if integral or derivative control is desired
		if (controlSignal > 20) controlSignal = 20; // limit the control signal to avoid wild overcorrections
		if (controlSignal < -20) controlSignal = -20;

		if (abs(controlSignal) < 10) {
			leftSpeed = TRAVELSPEED + 50 + (CAMERA_ORIENTATION * (-1))*(controlSignal);  // 2018 Camera Orientation: Connector at Front
			rightSpeed = TRAVELSPEED + 50 + (CAMERA_ORIENTATION * (+1))* (controlSignal); // Adjust the left and right wheel speeds
		}
		else {
			leftSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (-1))*controlSignal;  // 2018 Camera Orientation: Connector at Front
			rightSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (+1))* controlSignal; // Adjust the left and right wheel speeds
		}



		/*leftSpeed = TRAVELSPEED + ( CAMERA_ORIENTATION * (-1))*controlSignal;  // 2018 Camera Orientation: Connector at Front
		rightSpeed = TRAVELSPEED + ( CAMERA_ORIENTATION * (+1))* controlSignal; // Adjust the left and right wheel speeds */
		drive_speed(leftSpeed, rightSpeed); // set the drive_speed based upon the control signal
		delaycounter = delaycounter - 1;


	}

	IR_PLANT = 0;

	while (IR_PLANT < 1) {
		//printf("\x10");
		print(" \n %d", IR_PLANT);
		controlSignal = (int)(KP_HEADING * ((float)posError[4]) + (intError*KI_HEADING) + (derError[0] * KD_HEADING)); // posError[4] contains the most recent position error

																													   // if integral or derivative control is desired
		if (controlSignal > 20) controlSignal = 20; // limit the control signal to avoid wild overcorrections
		if (controlSignal < -20) controlSignal = -20;

		if (abs(controlSignal) < 10) {
			leftSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (-1))*(controlSignal);  // 2018 Camera Orientation: Connector at Front
			rightSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (+1))* (controlSignal); // Adjust the left and right wheel speeds
		}
		else {
			leftSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (-1))*controlSignal;  // 2018 Camera Orientation: Connector at Front
			rightSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (+1))* controlSignal; // Adjust the left and right wheel speeds
		}



		/*leftSpeed = TRAVELSPEED + ( CAMERA_ORIENTATION * (-1))*controlSignal;  // 2018 Camera Orientation: Connector at Front
		rightSpeed = TRAVELSPEED + ( CAMERA_ORIENTATION * (+1))* controlSignal; // Adjust the left and right wheel speeds */
		drive_speed(leftSpeed, rightSpeed); // set the drive_speed based upon the control signal

		check_IR_distance();

	}
   print("IR_PLANT: 0");

	leftSpeed = 0;
	rightSpeed = 0;


	drive_speed(leftSpeed, rightSpeed);

}

void go_straight_to_dropoff(void) {
	// Insert code here
	// navigation loop goes here


	int delaycounter;

	delaycounter = 50;

	while (delaycounter > 0) {
		controlSignal = (int)(KP_HEADING * ((float)posError[4]) + (intError*KI_HEADING) + (derError[0] * KD_HEADING)); // posError[4] contains the most recent position error

																														   // if integral or derivative control is desired
		if (controlSignal > 20) controlSignal = 20; // limit the control signal to avoid wild overcorrections
		if (controlSignal < -20) controlSignal = -20;

		if (abs(controlSignal) < 10) {
			leftSpeed = TRAVELSPEED + 50 + (CAMERA_ORIENTATION * (-1))*(controlSignal);  // 2018 Camera Orientation: Connector at Front
			rightSpeed = TRAVELSPEED + 50 + (CAMERA_ORIENTATION * (+1))* (controlSignal); // Adjust the left and right wheel speeds
		}
		else {
			leftSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (-1))*controlSignal;  // 2018 Camera Orientation: Connector at Front
			rightSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (+1))* controlSignal; // Adjust the left and right wheel speeds
		}



		/*leftSpeed = TRAVELSPEED + ( CAMERA_ORIENTATION * (-1))*controlSignal;  // 2018 Camera Orientation: Connector at Front
		rightSpeed = TRAVELSPEED + ( CAMERA_ORIENTATION * (+1))* controlSignal; // Adjust the left and right wheel speeds */
		drive_speed(leftSpeed, rightSpeed); // set the drive_speed based upon the control signal
		delaycounter = delaycounter - 1;


	}

	IR_PLANT_DROP = 0;

	while (IR_PLANT_DROP < 1) {
		//printf("\x10");
		print(" \n %d", IR_PLANT);
		controlSignal = (int)(KP_HEADING * ((float)posError[4]) + (intError*KI_HEADING) + (derError[0] * KD_HEADING)); // posError[4] contains the most recent position error

																													   // if integral or derivative control is desired
		if (controlSignal > 20) controlSignal = 20; // limit the control signal to avoid wild overcorrections
		if (controlSignal < -20) controlSignal = -20;

		if (abs(controlSignal) < 10) {
			leftSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (-1))*(controlSignal);  // 2018 Camera Orientation: Connector at Front
			rightSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (+1))* (controlSignal); // Adjust the left and right wheel speeds
		}
		else {
			leftSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (-1))*controlSignal;  // 2018 Camera Orientation: Connector at Front
			rightSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (+1))* controlSignal; // Adjust the left and right wheel speeds
		}



		/*leftSpeed = TRAVELSPEED + ( CAMERA_ORIENTATION * (-1))*controlSignal;  // 2018 Camera Orientation: Connector at Front
		rightSpeed = TRAVELSPEED + ( CAMERA_ORIENTATION * (+1))* controlSignal; // Adjust the left and right wheel speeds */
		drive_speed(leftSpeed, rightSpeed); // set the drive_speed based upon the control signal

		check_IR_drop();

	}
   	print("IR_PLANT: 0");

	leftSpeed = 0;
	rightSpeed = 0;


	drive_speed(leftSpeed, rightSpeed);

}

void go_to_line(void) {

	unsigned int leftBits; //All of the bits read in by the line following camera from center-left -> left
	unsigned int rightBits; //All of the bits read in by the line following camera from center-right -> right

	leftBits = countSetBits(binaryImage[3]) + countSetBits(binaryImage[2]);
	rightBits = countSetBits(binaryImage[1]) + countSetBits(binaryImage[0]);

	//

	while (leftBits + rightBits < 410) {
		// navigation loop goes here
		leftSpeed = TRAVELSPEED;
		rightSpeed = TRAVELSPEED;
		drive_speed(leftSpeed, rightSpeed);

		leftBits = countSetBits(binaryImage[3]) + countSetBits(binaryImage[2]);
		rightBits = countSetBits(binaryImage[1]) + countSetBits(binaryImage[0]);


	}


	leftSpeed = 0;
	rightSpeed = 0;
	drive_speed(leftSpeed, rightSpeed);



}

void rotate_robot_backwards(int Cwise_or_CCwise){
	// Turns robot 90 deg to left or right
	// Input can either be a -1 (counterclockwise) or +1 (clockwise)

	int leftSpeed, rightSpeed;
	float axel_length; // Distance (mm) between wheels
	float ticks_per_mm; // Ticks per mm defined by abdrive.h
	float turn_speed; // Time (s) to complete the 90 degree turn

	axel_length = 140;
	ticks_per_mm = 3.25;
	turn_speed = 1.5;

	leftSpeed = (3.14 * (axel_length / 4.0) * (1.0 / turn_speed) * (1.0 / ticks_per_mm));
	rightSpeed = (3.14 * (axel_length / 4.0) * (1.0 / turn_speed) * (1.0 / ticks_per_mm));

	//print("\n %d %d", leftSpeed, rightSpeed);


	if (Cwise_or_CCwise > 0) {
		leftSpeed = leftSpeed * 0.5;
		rightSpeed = rightSpeed * -1; // Slows down the inner wheel for a smoother turn
	}

	if (Cwise_or_CCwise < 0) {
		leftSpeed = leftSpeed * -1; // Slows down the inner wheel for a smoother turn
		rightSpeed = rightSpeed * 0.5;
	}

	drive_speed(leftSpeed, rightSpeed);

	pause(turn_speed * 1250.0);  // Slightly increased the turning time because it wasnt rotating fully

	leftSpeed = 0;
	rightSpeed = 0;

	drive_speed(leftSpeed, rightSpeed);

}

void rotate_robot_forwards(int Cwise_or_CCwise) {
	// Turns robot 90 deg to left or right
	// Input can either be a -1 (counterclockwise) or +1 (clockwise)

	int leftSpeed, rightSpeed;
	float axel_length; // Distance (mm) between wheels
	float ticks_per_mm; // Ticks per mm defined by abdrive.h
	float turn_speed; // Time (s) to complete the 90 degree turn

	axel_length = 140;
	ticks_per_mm = 3.25;
	turn_speed = 1.5;

	leftSpeed = (3.14 * (axel_length / 4.0) * (1.0 / turn_speed) * (1.0 / ticks_per_mm));
	rightSpeed = (3.14 * (axel_length / 4.0) * (1.0 / turn_speed) * (1.0 / ticks_per_mm));

	//print("\n %d %d", leftSpeed, rightSpeed);


	if (Cwise_or_CCwise > 0) {
		leftSpeed = leftSpeed * 1.5;
		rightSpeed = rightSpeed * .75; // Slows down the inner wheel for a smoother turn
	}

	if (Cwise_or_CCwise < 0) {
		leftSpeed = leftSpeed * .75; // Slows down the inner wheel for a smoother turn
		rightSpeed = rightSpeed * 1.5;
	}

	drive_speed(leftSpeed, rightSpeed);

	pause(turn_speed * 1250.0);  // Slightly increased the turning time because it wasnt rotating fully

	leftSpeed = 0;
	rightSpeed = 0;

	drive_speed(leftSpeed, rightSpeed);

}

void check_color(void){

  int counter;
  int CS_RED_VOLTS;
  int CS_GREEN_VOLTS;

  counter = 10;
  CS_RED_VOLTS = 0.0;
  CS_GREEN_VOLTS = 0.0;

  while (counter > 0) {

    CS_RED_VOLTS = input(CS_PIN_RED) + CS_RED_VOLTS;
    CS_GREEN_VOLTS = input(CS_PIN_GREEN) + CS_GREEN_VOLTS;

    counter = counter - 1;

  }

  if (CS_RED_VOLTS > CS_GREEN_VOLTS) {

    COLOR = -1; //red plant

  }
  if (CS_RED_VOLTS < CS_GREEN_VOLTS) {

    COLOR = 1; //green plant

  }

  print("RED SUM: %d \n", CS_RED_VOLTS);
  print("GREEN SUM: %d \n", CS_GREEN_VOLTS);

}

void go_straight_check_color(void) {
  
  unsigned int leftBits; //All of the bits read in by the line following camera from center-left -> left
	unsigned int rightBits; //All of the bits read in by the line following camera from center-right -> right
  int COLOR_SUM;

	leftBits = countSetBits(binaryImage[3]) + countSetBits(binaryImage[2]);
	rightBits = countSetBits(binaryImage[1]) + countSetBits(binaryImage[0]);
 
  MASTER_COLOR = 0;
  COLOR_SUM = 0;

	while (leftBits + rightBits < 410) {
		// navigation loop goes here
		controlSignal = (int)(KP_HEADING * ((float)posError[4]) + (intError*KI_HEADING) + (derError[0] * KD_HEADING)); // posError[4] contains the most recent position error

																														   // if integral or derivative control is desired
		if (controlSignal > 20) controlSignal = 20; // limit the control signal to avoid wild overcorrections
		if (controlSignal < -20) controlSignal = -20;

		if (abs(controlSignal) < 10) {
			leftSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (-1))*(controlSignal);  // 2018 Camera Orientation: Connector at Front
			rightSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (+1))* (controlSignal); // Adjust the left and right wheel speeds
		}
		else {
			leftSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (-1))*controlSignal;  // 2018 Camera Orientation: Connector at Front
			rightSpeed = TRAVELSPEED + (CAMERA_ORIENTATION * (+1))* controlSignal; // Adjust the left and right wheel speeds
		}

		drive_speed(leftSpeed, rightSpeed); // set the drive_speed based upon the control signal
		print(" %d , %d ", leftBits, rightBits);
		leftBits = countSetBits(binaryImage[3]) + countSetBits(binaryImage[2]);
		rightBits = countSetBits(binaryImage[1]) + countSetBits(binaryImage[0]);
  
    check_color();
    COLOR_SUM = COLOR + COLOR_SUM;
    
	}

  if (COLOR_SUM > 0) {
    MASTER_COLOR = 1; //plant is green  
  }    
  if (COLOR_SUM < 0) {
    MASTER_COLOR = -1; //plant is red
  }    
  
	leftSpeed = 0;
	rightSpeed = 0;
	drive_speed(leftSpeed, rightSpeed);

}  

/*
void drop_plant(void) {
	// Insert code here
}
*/

/*
int go_straight_check_color_check_intersection(void) {
	// Insert code here
}
void turn_towards_intersection(void) {
	// Insert code here
}
*/

int main() {
	int j;
	int CURRENT_PLANT;
	print("Started\n");
	adc_init(21, 20, 19, 18);
	pause(500);
	for (j = 0; j < 5; j++) posError[j] = 0;  // initialize position error array
	//simpleterm_close();  // Close SimpleIDE Terminal for this core

  // setting up the exposure parameters for a snapshot
	expTime_ms = 10;
	low(PIN_CLK_C);      // CLK pin low
	cogstart(getImage, NULL, stack, sizeof(stack));
	cogstart(bitSearch, NULL, stack2, sizeof(stack2)); // start printImageToTerminal process in new cog
	//cogstart(printImageToTerminal, NULL, stack3, sizeof(stack3)); // start printImageToTerminal process in new cog

  //  the following code shows an optional Autoexposure routine.  It is typically not needed in the class room with constant lighting.
  /*
	for(expTime_ms = 5, mj = 0, expSum = 0; expTime_ms < 12; expTime_ms++) {
	  while(input(OSCILLOSCOPE) == 1); // wait for new image to be exposed
	  while(input(OSCILLOSCOPE) == 0); // wait for new image to be available
	  if(zeroSum > 7 && zeroSum < 14) { mj++; expSum += expTime_ms; }
	  if(mj == 8) break;  // Stop once 10 acceptable images are obtained
	 }
	if(mj) expTime_ms = expSum/mj; // calculate the average exposure time
	else expTime_ms = 7;
	*/

	set_board_direction();
	if (BOARD_DIRECTION < 0) {
		IR_PIN = IR_PIN_RIGHT;
	}
	if (BOARD_DIRECTION > 0) {
		IR_PIN = IR_PIN_LEFT;
	}

	//cogstart(check_IR_distance, NULL, stack3, sizeof(stack3));

	servo_angle(SERVO_PIN, 0);                         // P16 servo to 0 degrees

	pause(1000);

	CURRENT_PLANT = 8;


	while (1) {


		go_to_line();

		rotate_robot(-1 * BOARD_DIRECTION);

		while(CURRENT_PLANT > 0){

			go_straight_check_IR();

			pick_plant();

			go_straight_check_intersection();

			rotate_robot(BOARD_DIRECTION);

			go_straight_check_color();

			rotate_robot(BOARD_DIRECTION);

			go_straight_to_dropoff();

	    drop_plant();

			go_straight_check_intersection();

			rotate_robot(BOARD_DIRECTION);

			go_straight_check_intersection();

			rotate_robot_backwards(BOARD_DIRECTION);

			rotate_robot_forwards(BOARD_DIRECTION);

			CURRENT_PLANT = CURRENT_PLANT -1;

		}

		pause (10000);

		//blind_turn( BOARD_DIRECTION );

		//pause(3000);



	}
}
