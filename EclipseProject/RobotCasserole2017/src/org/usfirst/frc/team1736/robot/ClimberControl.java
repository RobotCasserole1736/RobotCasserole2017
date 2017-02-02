package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Spark;

public class ClimberControl {

	//Declare Motor Control
	Spark climbMotor1 = new Spark(RobotIOMap.CLIMBER_MOTOR1_PWM_PORT);
	Spark climbMotor2 = new Spark(RobotIOMap.CLIMBER_MOTOR2_PWM_PORT);
	
	//Climber Speed
	public ClimberControl(){
			
			//Init Motor to off
			climbMotor1.set(0.0);
			climbMotor2.set(0.0);
		}
	
	//Climber Control
	public void update(){
		double climb_speed = RobotState.climbEnable && RobotState.climbSpeedCmd >= 0.0 ? RobotState.climbSpeedCmd : 0.0;
		climbMotor1.set(climb_speed);
		climbMotor2.set(climb_speed);
	}
	
}
