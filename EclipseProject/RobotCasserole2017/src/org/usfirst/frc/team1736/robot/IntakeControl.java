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

import org.usfirst.frc.team1736.lib.Calibration.Calibration;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;


public class IntakeControl {
	
	private static IntakeControl intakeControl = null;
	
	//Declare Motor Control
	Spark intakeMotor = new Spark(RobotConstants.INTAKE_MOTOR_PWM_PORT);
	
	//Declare Extender Control
	Solenoid intakeHPExtend = new Solenoid(RobotConstants.INTAKE_HP_EXTEND_SOLENOID_PORT);
	Solenoid intakeLPExtend = new Solenoid(RobotConstants.INTAKE_LP_EXTEND_SOLENOID_PORT);
	
	//Declaring Intake Calibration
	Calibration intakeMotorFwdCmd = new Calibration("Ground Pickup Intake Motor Command", 1.0, 0.0, 1.0);
	Calibration intakeMotorRevCmd = new Calibration("Ground Pickup Eject Motor Command", -1.0, -1.0, 0.0);
	
	//Intake Speed
	private final double INTAKE_OFF = 0.0;
	private static double intakeSpeedCommand = 0;
	
	public static synchronized IntakeControl getInstance()
	{
		if(intakeControl == null)
			intakeControl = new IntakeControl();
		return intakeControl;
	}
	
	private IntakeControl(){
		
		//Init Motor to off
		intakeMotor.set(0.0);
		
		//Init Extender to extended (by default)
		IntakeExtend();
	}
	
	public void update(){
		OperatorController operatorControl = OperatorController.getInstance();
		if(operatorControl.getIntakeDesiredCmd()){
			intakeSpeedCommand = intakeMotorFwdCmd.get();
		}else if(operatorControl.getEjectDesiredCmd()){
			intakeSpeedCommand = intakeMotorRevCmd.get();
		}else{
			intakeSpeedCommand = INTAKE_OFF;
		}
		intakeMotor.set(intakeSpeedCommand);
	}
	
	public void IntakeExtend(){
		intakeHPExtend.set(true);
		intakeLPExtend.set(true);
	}
	
	public void IntakeRetract(){
		intakeHPExtend.set(false);
		intakeLPExtend.set(false);
	}
	
	public void runIntakeFwd(){
		intakeSpeedCommand = intakeMotorFwdCmd.get();
		intakeMotor.set(intakeSpeedCommand);
	}
	
	public double getCommandedIntakeSpeed()
	{
		return intakeSpeedCommand;
	}
}
