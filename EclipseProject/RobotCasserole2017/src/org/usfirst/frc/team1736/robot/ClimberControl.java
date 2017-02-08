package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Spark;

public class ClimberControl {
	
	private static ClimberControl climberControl = null;
	
	private static boolean isClimbEnabled = true; //always enabled for now for testing

	//Declare Motor Control
	Spark climbMotor1 = new Spark(RobotConstants.CLIMBER_MOTOR1_PWM_PORT);
	Spark climbMotor2 = new Spark(RobotConstants.CLIMBER_MOTOR2_PWM_PORT);
	
	public static synchronized ClimberControl getInstance()
	{
		if(climberControl == null)
			climberControl = new ClimberControl();
		return climberControl;
	}
	
	private ClimberControl(){
		//Init Motor to off
		climbMotor1.set(0.0);
		climbMotor2.set(0.0);
	}
	
	//Climber Control
	public void update(){
		double operatorClimbCmd = OperatorController.getInstance().getClimbSpeedCmd();
		double climb_speed = isClimbEnabled && operatorClimbCmd >= 0.0 ? operatorClimbCmd : 0.0;
		
		climbMotor1.set(Math.abs(climb_speed));
		climbMotor2.set(Math.abs(climb_speed));
	}
	
	public void setClimbEnabled(boolean isEnabled)
	{
		isClimbEnabled = isEnabled;
	}
	
	public boolean getClimbEnabled()
	{
		return isClimbEnabled;
	}
	
}
