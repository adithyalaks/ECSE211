package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;

	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving
														// forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	/*
	 * This method processes the distance read from the ultrasonic sensor onboard the robot, and corrects 
	 * the the robot's trajectory in a bang-bang fashion in response to any discrepancy between where the 
	 * robot actually is, and where the robot should actually be ( specified by the bandcenter).
	 * 
	 */
	public void processUSData(int distance) {
		this.distance = distance;
		// TODO: process a movement based on the us distance passed in
		// (BANG-BANG style)

		// This first if statement deals with cases in which the robot finds itself too close
		// to an obstacle, and allows for it to reverse and avoid an obstacle.
		if (this.distance < 10) {
			WallFollowingLab.leftMotor.setSpeed(motorLow);
			WallFollowingLab.rightMotor.setSpeed(motorHigh + 40);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.backward();
		}

		// This if statement deals with cases in which the robot's distance from the wall is
		// close enough to the bandcenter. In this case, the robot is made to move forward
		// by setting both wheels to be spinning at equal velocities.
		else if (Math.abs((this.distance - bandCenter)) < bandwidth) {
			WallFollowingLab.leftMotor.setSpeed(motorHigh);
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}

		// This if statement deals with cases in which the robot's distance from
		// the wall is closer than the bandcenter - bandwidth. In this case, the left
		// motor rotates at a higher rate than the right motor, allowing for the robot to move
		// outwards.
		else if ((this.distance - bandCenter) < 0) {
			WallFollowingLab.leftMotor.setSpeed(motorHigh + 40);
			WallFollowingLab.rightMotor.setSpeed(motorLow);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}

		// This if statement deals with cases in which the robot's distance from
		// the wall is further than the (bandcenter - bandwidth). In this case, the right
		// motor rotates at a higher rate than the left motor, allowing for the robot to move
		// inwards.
		else if ((this.distance - bandCenter) > 0) {
			WallFollowingLab.leftMotor.setSpeed(motorLow);
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}

	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
