package org.usfirst.frc.team1736.robot;

/*
 *******************************************************************************************
 * Copyright (C) 2017 FRC Team 1736 Robot Casserole - www.robotcasserole.org
 *******************************************************************************************
 *
 * This software is released under the MIT Licence - see the license.txt
 *  file in the root of this repo.
 *
 * Non-legally-binding statement from Team 1736:
 *  Thank you for taking the time to read through our software! We hope you
 *   find it educational and informative! 
 *  Please feel free to snag our software for your own use in whatever project
 *   you have going on right now! We'd love to be able to help out! Shoot us 
 *   any questions you may have, all our contact info should be on our website
 *   (listed above).
 *  If you happen to end up using our software to make money, that is wonderful!
 *   Robot Casserole is always looking for more sponsors, so we'd be very appreciative
 *   if you would consider donating to our club to help further STEM education.
 */

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
