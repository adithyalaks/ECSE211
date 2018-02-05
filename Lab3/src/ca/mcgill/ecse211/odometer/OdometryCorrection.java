/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;

public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;
	private OdometerData odoData;
	private EV3ColorSensor cSensor;
	private Sound sound;
	private final double TILE_SIZE = 30.48;
	private int[] numLinesPassed = { 0, 0 };
	private int NUM_TILES = 3;

	/**
	 * This is the default class constructor. An existing instance of the
	 * odometer is used. This is to ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection() throws OdometerExceptions {

		this.odometer = Odometer.getOdometer();
		// this.odoData = odometer.getOdometerData();
		this.cSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
		cSensor.setFloodlight(Color.WHITE);
	}

	/**
	 * Here is where the odometer correction code should be run.
	 * 
	 * The RGB sensor is repeatedly polled, and when it dips below a determined
	 * value, it triggers a correction to the odometer data, depending on the
	 * heading of the robot.
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;

		while (true) {
			correctionStart = System.currentTimeMillis();
			// TODO Trigger correction (When do I have information to correct?)
			float[] RGB = new float[3];
			cSensor.getRGBMode().fetchSample(RGB, 0);

			// This is the trigger condition, and the RGB values were determined
			// during testing.
			if ((RGB[0] < 0.12) && (RGB[1] < 0.12) && RGB[2] < 0.12) {
				sound.beep();
				// We poll the odometer data to determine the heading of the
				// robot.
				double[] currentData = odometer.getXYT();

				// If the robot is heading in the positive Y direction (0 - 90
				// degrees, and so
				// corrections are made to the odometer's Y value. We keep track
				// of the number
				// of lines passed in an array called NumLines Passed, and store
				// the lines passed
				// in the y-direction at index 1.
				if (currentData[2] < 80 && numLinesPassed[1] >= 0) {
					odometer.setY(numLinesPassed[1] * TILE_SIZE);
					if (numLinesPassed[1] < (NUM_TILES - 1)) {
						numLinesPassed[1]++;
					}
					LCD.drawString("YAxis Counter " + numLinesPassed[1], 0, 4);
				}

				// If the robot is heading in the positive X direction (90-180
				// degrees), and so corrections are made
				// to the odometer's X value. We keep track of the number of
				// lines passed in an array
				// called NumLines Passed, and store the lines passed in the
				// x-direction at index 0.
				else if (currentData[2] < 170 && numLinesPassed[0] >= 0) {
					odometer.setX(numLinesPassed[0] * TILE_SIZE);
					if (numLinesPassed[0] < (NUM_TILES - 1)) {
						numLinesPassed[0]++;
					}
					LCD.drawString("XAxis Counter - " + numLinesPassed[0], 0, 6);
				}

				// If the robot heads at a direction between 180-270 degrees, we
				// know the robot is heading in the negative
				// y direction. We therefore adjust the y-values and decrement
				// the numLinesPassed array at index 1 in order
				// to keep track of what value the odometer data should be
				// corrected to as it passes a given line.
				else if (currentData[2] < 260) {
					odometer.setY(numLinesPassed[1] * TILE_SIZE - 1.96);
					numLinesPassed[1]--;
					LCD.drawString("YAxis Counter - " + numLinesPassed[1], 0, 4);
				}
				// If the robots heads at a direction between 270-360 degrees,
				// the robot is heading in the negative
				// x-direction. We therefore adjust the x-value displayed by the
				// odometer, and decrement the numLinesPassed
				// array at index 0. This is done to keep track of how much the
				// odometer values ought to be adjusted by as it
				// passes a given line, and helps keep track of where the robot
				// is.
				else if (currentData[2] < 350) {
					odometer.setX(numLinesPassed[0] * TILE_SIZE - 1.55);
					numLinesPassed[0]--;
					LCD.drawString("XAxis Counter - " + numLinesPassed[0], 0, 6);
				}
			}

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}
}
