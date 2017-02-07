package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.Sensors.ADXRS453_Gyro;

/**
 * This class is intended to just be a simple singleton wrapper around our gyro.  It can implement methods specific to our gyro (such as inverting the angle)
 *
 */
public class Gyro {
	private static Gyro gyro;
	private static ADXRS453_Gyro adxrs453;
	
	public static synchronized Gyro getInstance()
	{
		if(gyro == null)
			gyro = new Gyro();
		return gyro;
	}
	
	private Gyro()
	{
		adxrs453 = new ADXRS453_Gyro();
	}
	
	public double getAngle()
	{
		return -adxrs453.getAngle();
	}
	
	public void reset()
	{
		adxrs453.reset();
	}
	
}
