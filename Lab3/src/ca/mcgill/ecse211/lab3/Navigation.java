package ca.mcgill.ecse211.lab3;

import java.util.Arrays;	
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.Sound;
import lejos.hardware.lcd.*;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;

public class Navigation implements Runnable {
	
	private Odometer odometer;
	private Sound sound;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	
	private float[] usData;
	private SampleProvider us;
	private static final int MIN_DISTANCE_TO_WALL= 5;
	
	private int [][] wayPoints = new int [5][2];
	
	private static final int UPDATE_PERIOD = 1000; 
	private static final double TILE_SIZE = 30.48;  
	private static final int FORWARD_SPEED = 150;
	private static final int ROTATE_SPEED = 100;
	private static double wheelRadius;
	private static double track; 
	private double acceptableDistanceError = 3;
	
	public Navigation (Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			int[][] wayPoints, double wheelRadius, double track, SampleProvider usDistance) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.wayPoints = wayPoints; 
		this.wheelRadius = wheelRadius;
		this.track = track;
		this.us = usDistance;
		this.usData = new float[this.us.sampleSize()];
		//this.leftMotor.setAcceleration(300);
		//this.rightMotor.setAcceleration(300);
		
		
	}
	
	private int usPoller() {
		us.fetchSample(usData, 0);
		int distance = (int) (usData[0]*100);
		return distance;
	}
	
	private void turnRight() {
	      leftMotor.setSpeed(ROTATE_SPEED);
	      rightMotor.setSpeed(ROTATE_SPEED);

	      leftMotor.rotate(convertAngle(wheelRadius, track, 90.0), true);
	      rightMotor.rotate(-convertAngle(wheelRadius, track, 90.0), false);
	}
	
	private void turnLeft() {
	      leftMotor.setSpeed(ROTATE_SPEED);
	      rightMotor.setSpeed(ROTATE_SPEED);

	      rightMotor.rotate(convertAngle(wheelRadius, track, 90.0), true);
	      leftMotor.rotate(-convertAngle(wheelRadius, track, 90.0), false);
	}
	
	private void avoidObstacle() {
		double [] position = odometer.getXYT();
		boolean turnedRight;
		
		if (position [2] < 180 ) {
			if (position[0] < TILE_SIZE) {
				turnRight();
				turnedRight = true;
			}
			else { 
				turnLeft();
				turnedRight = false;
			}
		}
		else  {
			if (position[0] > TILE_SIZE ) {
				turnRight();
				turnedRight = true;
			}
			else {
				turnLeft();
				turnedRight = false;
			}
		}
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(wheelRadius,TILE_SIZE),true);
		rightMotor.rotate(convertDistance(wheelRadius,TILE_SIZE),false);
		
		if (turnedRight) {
			turnLeft();
		}
		
		else turnRight();
	}
	
	private void computeHeadingAndRotate(double x, double y) {
		double [] position = odometer.getXYT();
		double dX = x - position[0];
		double dY = y - position[1];
		
		double requiredHeading = Math.atan(Math.abs(dX)/Math.abs(dY))*(180/Math.PI);
		
		if ((dX > 0) && (dY < 0 )) requiredHeading = 180 - requiredHeading;
		
		else if ((dX < 0) && (dY < 0)) requiredHeading += 180;
		
		else if((dX < 0) && (dY > 0)) requiredHeading = 360 -requiredHeading;
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		turnTo(requiredHeading);
		
	}
		
	private void travelTo(double x, double y){
		boolean haveReached = false;
		
		computeHeadingAndRotate(x,y);
		
		while (!haveReached) {
			
		int distanceToObstacle = usPoller();
		
		if (distanceToObstacle < MIN_DISTANCE_TO_WALL ) {
			avoidObstacle();
			computeHeadingAndRotate(x,y);
			continue;
		}
		double [] position = odometer.getXYT();
		double dX = x - position[0];
		double dY = y - position[1];
		double distanceToGoal = Math.sqrt(dX*dX + dY*dY);
		
		if (distanceToGoal < acceptableDistanceError) {
			haveReached = true;
			break;
		}
		
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.forward();
		
		}
		Sound.beep();
	}
	
	private void turnTo(double theta) {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		double currentHeading = odometer.getXYT()[2];
		double dHeading = theta - currentHeading;
		if (Math.abs(dHeading) < 1) return;
		
		if (dHeading > 180) {
			dHeading = dHeading - 360;			
		}
		
		else if (dHeading < -180) {
			dHeading = 360 + dHeading;
		}
		
		if (dHeading > 0) {
			 leftMotor.rotate(convertAngle(wheelRadius, track, dHeading), true);
		     rightMotor.rotate(-convertAngle(wheelRadius, track, dHeading), false);
		}
		
		else if(dHeading < 0) {
			rightMotor.rotate(convertAngle(wheelRadius, track, Math.abs(dHeading)), true);
		    leftMotor.rotate(-convertAngle(wheelRadius, track, Math.abs(dHeading)), false);
		}
	}
	
	
	public boolean isNavigating() {
		return true;
	}
	
	private static int convertDistance(double radius, double distance) {
		    return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	  private static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	@Override
	public void run() {
		for (int[] waypoint :wayPoints) {
			travelTo(TILE_SIZE*waypoint[0],TILE_SIZE*waypoint[1]);
		
		}
		leftMotor.stop();
		rightMotor.stop();
	}

}
