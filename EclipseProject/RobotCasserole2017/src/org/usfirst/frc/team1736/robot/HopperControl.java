package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Victor;

public class HopperControl {
	//Declare Motor Control
	Victor hopMotor = new Victor(1);
	
	//Hopper Speed
	double hopSpeedOn = 0.5;
	double hopSpeedOff = 0.0;
	public HopperControl(){
		
		//Init Motor to off
		RobotState.hopperMotorCmd = 0.0;
		hopMotor.set(RobotState.hopperMotorCmd);

	}
	
	
	
	public void setSwitch(boolean on) {
		if(on){
			RobotState.hopperMotorCmd = hopSpeedOn;
		}else{
			RobotState.hopperMotorCmd = hopSpeedOff;
		}
		hopMotor.set(RobotState.hopperMotorCmd);
	}
}