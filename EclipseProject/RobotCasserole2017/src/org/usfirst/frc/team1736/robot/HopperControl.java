package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.Calibration.Calibration;

import edu.wpi.first.wpilibj.Spark;

public class HopperControl {
	//Declare Motor Control
	Spark hopMotor = new Spark(RobotConstants.HOPPER_MOTOR_PWM_PORT);
	
	//Declaring Hopper Calibration
	Calibration hopperMotorCmd = new Calibration("Hopper Feed Motor Command", 0.5, 0.0, 1.0);
	
	//Hopper Speed
	double hopSpeedOff = 0.0;
	public HopperControl(){
		
		//Init Motor to off
		hopMotor.set(0.0);

	}
	
	
	
	public void update() {

		if (RobotState.opHopperFwdDesired){
			RobotState.hopperMotorCmd = hopperMotorCmd.get();
		}else if (RobotState.opHopperRwdDesired){
			RobotState.hopperMotorCmd = hopperMotorCmd.get() * -1.0;
		}
		else if(RobotState.hopperFeedCmd){
			RobotState.hopperMotorCmd = hopperMotorCmd.get();
		}else{
			RobotState.hopperMotorCmd = hopSpeedOff;
		}
		hopMotor.set(RobotState.hopperMotorCmd);
	}
}
