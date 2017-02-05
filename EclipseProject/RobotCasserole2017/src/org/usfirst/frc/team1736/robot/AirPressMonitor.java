package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class AirPressMonitor {
	private static AnalogInput psensor = new AnalogInput(RobotConstants.AIR_PRESSURE_SENSOR_PORT);
	
	public double getPress(){
		return ((psensor.getVoltage()/5.0)-0.1)*150.0/0.8;
	}

}
