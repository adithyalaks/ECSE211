package ca.mcgill.ecse211.lab3;

import java.awt.Choice;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;

public class Lab3 {
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 11.8;
	
	public static void main(String[] args) throws OdometerExceptions {
		
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		
		SensorModes usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));
		SampleProvider usDistance = usSensor.getMode("Distance");
		
		Display odometryDisplay = new Display(lcd);
		
		int [][] wayPoints = {{1,0},{2,1},{2,2},{0,2},{1,1}};
		
		Navigation navigation = new Navigation(odometer,leftMotor,rightMotor, wayPoints, WHEEL_RAD,TRACK,usDistance);
		
		int buttonChoice;
		do {
			lcd.clear();
			
			lcd.drawString("Press the right button to start navigating.", 0, 0);
			lcd.drawString("Press any other button the escape button to exit.", 0,4);
			
			buttonChoice = Button.waitForAnyPress();
			
		} while (buttonChoice != Button.ID_ESCAPE && buttonChoice != Button.ID_RIGHT);
		
		if (buttonChoice == Button.ID_ESCAPE) System.exit(0);
		
		else if (buttonChoice == Button.ID_RIGHT) {
			
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			
			Thread naviThread = new Thread(navigation);
			naviThread.start();
			
		}
		
		while (Button.waitForAnyPress()!= Button.ID_ESCAPE);
		System.exit(0);
				
}
}
