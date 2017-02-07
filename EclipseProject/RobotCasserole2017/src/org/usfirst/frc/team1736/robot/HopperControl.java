package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.Calibration.Calibration;

import edu.wpi.first.wpilibj.Spark;

public class HopperControl {
	private static HopperControl hopperControl = null;
	
	//Declare Motor Control
	private Spark hopMotor = new Spark(RobotConstants.HOPPER_MOTOR_PWM_PORT);
	
	//Declaring Hopper Calibration
	Calibration hopperMotorCmd = new Calibration("Hopper Feed Motor Command", 0.5, 0.0, 1.0);
	
	private double motorCmd = 0;
	
	//Hopper Speed
	private double hopSpeedOff = 0.0;
	
	public static synchronized HopperControl getInstance()
	{
		if(hopperControl == null)
			hopperControl = new HopperControl();
		return hopperControl;
	}
	
	private HopperControl(){
		
		//Init Motor to off
		hopMotor.set(0.0);
		
	}
	
	public void update() {
		if(ShotControl.getInstance().getHopperFeedCmd()){
			motorCmd = hopperMotorCmd.get();
		}else{
			motorCmd = hopSpeedOff;
		}
		hopMotor.set(motorCmd);
	}
	
	public double getHopperMotorCmd()
	{
		return motorCmd;
	}
}
