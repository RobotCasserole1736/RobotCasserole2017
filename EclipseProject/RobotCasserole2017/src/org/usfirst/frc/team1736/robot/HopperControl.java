package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.Calibration.Calibration;

import edu.wpi.first.wpilibj.Victor;

public class HopperControl {
	//Declare Motor Control
	Victor hopMotor = new Victor(1);
	
	//Declaring Hopper Calibration
	Calibration hopperMotorCmd = new Calibration("hopperMotorCmd", 0.5, 0.0, 1.0);
	
	//Hopper Speed
	double hopSpeedOff = 0.0;
	public HopperControl(){
		
		//Init Motor to off
		RobotState.hopperMotorCmd = 0.0;
		hopMotor.set(RobotState.hopperMotorCmd);
		

	}
	
	
	
	public void setSwitch(boolean on) {
		if(on){
			RobotState.hopperMotorCmd = hopperMotorCmd.get();
		}else{
			RobotState.hopperMotorCmd = hopSpeedOff;
		}
		hopMotor.set(RobotState.hopperMotorCmd);
	}
}