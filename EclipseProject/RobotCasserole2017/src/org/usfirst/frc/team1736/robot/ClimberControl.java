package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Victor;

public class ClimberControl {

	//Declare Motor Control
	Victor climbMotor = new Victor(1);
	
	//Climber Speed
	public ClimberControl(){
			
			//Init Motor to off
			climbMotor.set(0.0);
		}
	
	//Climber Control
	public void update(){
		climbMotor.set(RobotState.climbEnable && RobotState.climbSpeedCmd >= 0.0 ? RobotState.climbSpeedCmd : 0.0);
	}
	
}
