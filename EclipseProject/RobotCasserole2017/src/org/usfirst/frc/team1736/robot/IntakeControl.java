package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.Calibration.Calibration;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;


public class IntakeControl {
	
	private static IntakeControl intakeControl = null;
	
	//Declare Motor Control
	Spark intakeMotor = new Spark(RobotConstants.INTAKE_MOTOR_PWM_PORT);
	
	//Declare Extender Control
	Solenoid intakeExtend = new Solenoid(RobotConstants.INTAKE_EXTEND_SOLENOID_PORT);
	
	//Declaring Intake Calibration
	Calibration intakeMotorFwdCmd = new Calibration("Ground Pickup Intake Motor Command", 0.5, 0.0, 1.0);
	Calibration intakeMotorRevCmd = new Calibration("Ground Pickup Eject Motor Command", -0.5, -1.0, 0.0);
	
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
		
		//Init Extender to retracted
		intakeExtend.set(true);
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
		intakeExtend.set(true);
	}
	
	public double getCommandedIntakeSpeed()
	{
		return intakeSpeedCommand;
	}
}
