/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.hardware.Sound;


public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private EV3ColorSensor cSensor;
  private Sound sound;
  

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();
    this.cSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
    cSensor.setFloodlight(Color.WHITE);
  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    
    
  
    while (true) {
      correctionStart = System.currentTimeMillis();

      
      // TODO Trigger correction (When do I have information to correct?)
      float [] RGB = new float[3];
      cSensor.getRGBMode().fetchSample(RGB,0);
      
      if ((RGB[0] < 0.12) && (RGB[1] < 0.12) && RGB[2] < 0.12 ) {
      	sound.beep();
      	
      }

      // TODO Calculate new (accurate) robot position

      // TODO Update odometer with new calculated (and more accurate) vales

      //odometer.setXYT(0.3, 19.23, 5.0);

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