package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.Sensors.ADXRS453_Gyro;

/**
 * This class is intended to just be a simple singleton wrapper around our gyro.  It can implement methods specific to our gyro (such as inverting the angle)
 *
 */
public class Gyro {
	private static Gyro gyro;
	private static ADXRS453_Gyro adxrs453;
	private static int angleOffset;
	
	public static synchronized Gyro getInstance()
	{
		if(gyro == null)
			gyro = new Gyro();
		return gyro;
	}
	
	private Gyro()
	{
		adxrs453 = new ADXRS453_Gyro();
		angleOffset = 0;
	}
	
	public double getAngle()
	{
		return angleOffset - adxrs453.getAngle();
	}
	
	public void reset()
	{
		adxrs453.reset();
	}
	
	public void setAngleOffset(int angle)
	{
		angleOffset = angle;
	}
	public int getAngleOffset()
	{
		return angleOffset;
	}
}
