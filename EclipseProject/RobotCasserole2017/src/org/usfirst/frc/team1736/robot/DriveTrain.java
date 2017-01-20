package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import org.usfirst.frc.team1736.robot.RobotState;
import org.usfirst.frc.team1736.lib.Calibration.Calibration;
import org.usfirst.frc.team1736.lib.HAL.Xbox360Controller;
public class DriveTrain{
	
	RobotDrive myDrive;

	Victor frontLeftMotor;
	Victor frontRightMotor;
	Victor rearLeftMotor;
	Victor rearRightMotor;
	
	Encoder frontLeftEncoder;
	Encoder frontRightEncoder;
	Encoder rearLeftEncoder;
	Encoder rearRightEncoder;
	
	Calibration fieldOrientedCtrl;
	
	public static final double DRIVETRAIN_WHEELS_REV_PER_TICK = 1.0;
	
	public static final double DRIVETRAIN_WHEELS_RADIUS_FT= 4.0/2.0/12.0; //4 inch diameter wheel, converted to radius in feet
	

	
	public DriveTrain() {
		//Setup Drivetrain with motors and such
		frontLeftMotor  = new Victor(RobotIOMap.DRIVETRAIN_FRONT_LEFT_MOTOR);
    	frontRightMotor = new Victor(RobotIOMap.DRIVETRAIN_FRONT_RIGHT_MOTOR);
    	rearLeftMotor   = new Victor(RobotIOMap.DRIVETRAIN_REAR_LEFT_MOTOR);
    	rearRightMotor  = new Victor(RobotIOMap.DRIVETRAIN_REAR_RIGHT_MOTOR);
    	myDrive = new RobotDrive(frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor);
    	
    	//Set up configurable field-oriented control
    	fieldOrientedCtrl = new Calibration("Enable Field-Oriented Control", 0.0, 0.0, 1.0);
    	
    	//set up encoders
    	frontLeftEncoder  = new Encoder(RobotIOMap.DRIVETRAIN_FRONT_LEFT_ENCODER_A, RobotIOMap.DRIVETRAIN_FRONT_LEFT_ENCODER_B,  false);
    	frontRightEncoder = new Encoder(RobotIOMap.DRIVETRAIN_FRONT_RIGHT_ENCODER_A,RobotIOMap.DRIVETRAIN_FRONT_RIGHT_ENCODER_B, false);
    	rearLeftEncoder   = new Encoder(RobotIOMap.DRIVETRAIN_REAR_LEFT_ENCODER_A,  RobotIOMap.DRIVETRAIN_REAR_LEFT_ENCODER_B,   false);
    	rearRightEncoder  = new Encoder(RobotIOMap.DRIVETRAIN_REAR_RIGHT_ENCODER_A, RobotIOMap.DRIVETRAIN_REAR_RIGHT_ENCODER_B,  false);
    	
    	frontLeftEncoder.setDistancePerPulse(DRIVETRAIN_WHEELS_REV_PER_TICK);
    	frontRightEncoder.setDistancePerPulse(DRIVETRAIN_WHEELS_REV_PER_TICK);
    	rearLeftEncoder.setDistancePerPulse(DRIVETRAIN_WHEELS_REV_PER_TICK);
    	rearRightEncoder.setDistancePerPulse(DRIVETRAIN_WHEELS_REV_PER_TICK);
	}
	
	
	/**
	 * Reads the current speed and total distance of each wheel on the drivetrain. 
	 * Should be called each periodic loop.
	 */
	public void readEncoders(){
		
		//getRate returns in per seconds, so we need to convert
		RobotState.frontLeftWheelVelocity_rpm  = frontLeftEncoder.getRate()*60.0;
		RobotState.frontRightWheelVelocity_rpm = frontRightEncoder.getRate()*60.0;
		RobotState.rearLeftWheelVelocity_rpm   = rearLeftEncoder.getRate()*60.0;
		RobotState.rearRightWheelVelocity_rpm  = rearRightEncoder.getRate()*60.0;
		
		//Get distance returns in total revolutions, so convert to ft with math
		RobotState.frontLeftWheelDistance_ft  = frontLeftEncoder.getDistance()*2.0*Math.PI*DRIVETRAIN_WHEELS_RADIUS_FT;
		RobotState.frontRightWheelDistance_ft = frontRightEncoder.getDistance()*2.0*Math.PI*DRIVETRAIN_WHEELS_RADIUS_FT;
		RobotState.rearLeftWheelDistance_ft   = rearLeftEncoder.getDistance()*2.0*Math.PI*DRIVETRAIN_WHEELS_RADIUS_FT;
		RobotState.rearRightWheelDistance_ft  = rearRightEncoder.getDistance()*2.0*Math.PI*DRIVETRAIN_WHEELS_RADIUS_FT;
		
	}
	
	/**
	 * Sets all encoder distances back to zero.
	 */
	public void resetEncoders(){
		frontLeftEncoder.reset();
		frontRightEncoder.reset();
		rearLeftEncoder.reset();
		rearRightEncoder.reset();
	}
	

	public void autonomousControl() {
		if(RobotState.visionAlignmentDesiried){
			if(RobotState.visionAlignmentPossible){
				//If we're in autonomous and vision alignment is desired (and possible), use the vision commands with no strafe
				myDrive.mecanumDrive_Cartesian(RobotState.visionDtFwdRevCmd, 0, RobotState.visionDtRotateCmd, 0);
			} else {
				//If the auto routine wanted vision but we can't find a target, give up and stay still.
				myDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
			}

		} else {
			//If we're in autonomous but don't need vision alignment, use the normal autonomous motor commands
			myDrive.mecanumDrive_Cartesian(RobotState.autonDtFwdRevCmd, RobotState.autonDtrStrafeCmd, RobotState.autonDtRotateCmd, 0);
		}
		
		updateMotorCmds();
	}

	public void operatorControl() {
			
		if(RobotState.visionAlignmentDesiried & RobotState.visionAlignmentPossible){
			//For operator control, vision assist, get commands from the vision subsystem (although the driver may still strafe)
			myDrive.mecanumDrive_Cartesian(RobotState.visionDtFwdRevCmd, RobotState.driverStrafeCmd, RobotState.visionDtRotateCmd, 0);
		} else if(fieldOrientedCtrl.get() == 0.0){
			//For operator control, non-field oriented, and no vision assist, get all commands from driver 
			myDrive.mecanumDrive_Cartesian(RobotState.driverFwdRevCmd, RobotState.driverStrafeCmd, RobotState.driverRotateCmd, 0);
		} else {
			//For operator control, field oriented, and no vision assist, get all commands from driver along with gyro angle
			myDrive.mecanumDrive_Cartesian(RobotState.driverFwdRevCmd, RobotState.driverStrafeCmd, RobotState.driverRotateCmd, RobotState.robotPoseAngle_deg);
		}
		
		updateMotorCmds();

	}
	
	
	private void updateMotorCmds(){		
		//Update Motor Commands
		RobotState.frontLeftDriveMotorCmd  =  getFLDriveMotorCmd();
		RobotState.frontRightDriveMotorCmd =  getFRDriveMotorCmd();
		RobotState.rearLeftDriveMotorCmd   =  getRLDriveMotorCmd();
		RobotState.rearRightDriveMotorCmd  =  getRRDriveMotorCmd();
	}

	public double getFLDriveMotorCmd() {
		return frontLeftMotor.get();
	}
	
	public double getFRDriveMotorCmd() {
		return frontRightMotor.get();
	}
	
	public double getRLDriveMotorCmd() {
		return rearLeftMotor.get();
	}

	public double getRRDriveMotorCmd() {
		return rearRightMotor.get();
	}



}