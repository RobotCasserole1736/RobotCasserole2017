package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.Calibration.Calibration;

import edu.wpi.first.wpilibj.Victor;

public class IntakeControl {
	
	//Declare Motor Control
	Victor intakeMotor = new Victor(RobotIOMap.INTAKE_MOTOR_PWM_PORT);
	
	//Declaring Intake Calibration
	Calibration intakeMotorFwdCmd = new Calibration("Forward Intake Motor Command", 0.5, 0.0, 1.0);
	Calibration intakeMotorRevCmd = new Calibration("Reverse Intake Motor Command", -0.5, -1.0, 0.0);
	
	//Intake Speed
	double intakeOff = 0.0;
	public IntakeControl(){
		
		//Init Motor to off
		intakeMotor.set(0.0);
		

	}
	public void update() {
		if(RobotState.opIntakeDesired){
			RobotState.intakeSpeedCmd = intakeMotorFwdCmd.get();
		}else if(RobotState.opEjectDesired){
			RobotState.intakeSpeedCmd = intakeMotorRevCmd.get();
		}else{
			RobotState.intakeSpeedCmd = intakeOff;
		}
		intakeMotor.set(RobotState.intakeSpeedCmd);
	}
	
}
