package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import org.usfirst.frc.team1736.robot.RobotState;
import org.usfirst.frc.team1736.lib.Calibration.Calibration;
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
	
	DriveTrainWheelSpeedPI frontLeftAutonCtrl;
	DriveTrainWheelSpeedPI frontRightAutonCtrl;
	DriveTrainWheelSpeedPI rearLeftAutonCtrl;
	DriveTrainWheelSpeedPI rearRightAutonCtrl;
	
	Calibration dtPGainCal;
	Calibration dtFGainCal;
	Calibration dtIGainCal;
	
	Calibration fieldOrientedCtrl;
	
	boolean runningClosedLoop;
	
	public static final double DRIVETRAIN_WHEELS_REV_PER_TICK = 1.0/2048.0;
	
	public static final double DRIVETRAIN_WHEELS_RADIUS_FT= 4.0/2.0/12.0; //4 inch diameter wheel, converted to radius in feet
	

	
	public DriveTrain() {
		//Setup Drivetrain with motors and such
		frontLeftMotor  = new Victor(RobotIOMap.DRIVETRAIN_FRONT_LEFT_MOTOR);
    	frontRightMotor = new Victor(RobotIOMap.DRIVETRAIN_FRONT_RIGHT_MOTOR);
    	rearLeftMotor   = new Victor(RobotIOMap.DRIVETRAIN_REAR_LEFT_MOTOR);
    	rearRightMotor  = new Victor(RobotIOMap.DRIVETRAIN_REAR_RIGHT_MOTOR);
    	
    	//Set inversion on drivetrain motors (opposite sides need to be flipped in sign so positive command yeilds positive motion)
    	frontLeftMotor.setInverted(false);
    	frontRightMotor.setInverted(false);
    	rearLeftMotor.setInverted(true);
    	rearRightMotor.setInverted(true);
    	
    	myDrive = new RobotDrive(frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor);
    	
    	//Set up calibratable values
    	fieldOrientedCtrl = new Calibration("Enable Field-Oriented Control", 0.0, 0.0, 1.0);
    	dtPGainCal = new Calibration ("DT Auton Velocity P Gain", 0.0);
    	dtFGainCal = new Calibration ("DT Auton Velocity F Gain", 0.0005);
    	dtIGainCal = new Calibration ("DT Auton Velocity I Gain", 0.0);
    	
    	//set up encoders
    	frontLeftEncoder  = new Encoder(RobotIOMap.DRIVETRAIN_FRONT_LEFT_ENCODER_A,  RobotIOMap.DRIVETRAIN_FRONT_LEFT_ENCODER_B,  false);
    	frontRightEncoder = new Encoder(RobotIOMap.DRIVETRAIN_FRONT_RIGHT_ENCODER_A, RobotIOMap.DRIVETRAIN_FRONT_RIGHT_ENCODER_B, false);
    	rearLeftEncoder   = new Encoder(RobotIOMap.DRIVETRAIN_REAR_LEFT_ENCODER_A,   RobotIOMap.DRIVETRAIN_REAR_LEFT_ENCODER_B,   false);
    	rearRightEncoder  = new Encoder(RobotIOMap.DRIVETRAIN_REAR_RIGHT_ENCODER_A,  RobotIOMap.DRIVETRAIN_REAR_RIGHT_ENCODER_B,  false);
    	
    	//Note minus signs to invert right side of drivetrain
    	frontLeftEncoder.setDistancePerPulse(DRIVETRAIN_WHEELS_REV_PER_TICK);
    	frontRightEncoder.setDistancePerPulse(-DRIVETRAIN_WHEELS_REV_PER_TICK);
    	rearLeftEncoder.setDistancePerPulse(-DRIVETRAIN_WHEELS_REV_PER_TICK);
    	rearRightEncoder.setDistancePerPulse(DRIVETRAIN_WHEELS_REV_PER_TICK);
    	
    	
    	
    	//Set up autonomous PI controllers
    	frontLeftAutonCtrl = new DriveTrainWheelSpeedPI(frontLeftMotor,  frontLeftEncoder,  dtFGainCal, dtPGainCal, dtIGainCal);
    	frontRightAutonCtrl = new DriveTrainWheelSpeedPI(frontRightMotor, frontRightEncoder, dtFGainCal, dtPGainCal, dtIGainCal);
    	rearLeftAutonCtrl = new DriveTrainWheelSpeedPI(rearLeftMotor,   rearLeftEncoder,   dtFGainCal, dtPGainCal, dtIGainCal);
    	rearRightAutonCtrl = new DriveTrainWheelSpeedPI(rearRightMotor,  rearRightEncoder,  dtFGainCal, dtPGainCal, dtIGainCal);
    	
    	runningClosedLoop = false;
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
	public void resetAllEncoders(){
		frontLeftEncoder.reset();
		frontRightEncoder.reset();
		rearLeftEncoder.reset();
		rearRightEncoder.reset();
	}
	
	/**
	 * Resets all autonomous velocity PI controller error integrators to zero.
	 */
	public void resetAllIntegrators(){
		frontLeftAutonCtrl.resetIntegrators();
		frontRightAutonCtrl.resetIntegrators();
		rearLeftAutonCtrl.resetIntegrators();
		rearRightAutonCtrl.resetIntegrators();
	}
	
	/**
	 * Update calibration values for all velocity PI controllers.
	 */
	public void updateAllCals(){
		frontLeftAutonCtrl.updateCal();
		frontRightAutonCtrl.updateCal();
		rearLeftAutonCtrl.updateCal();
		rearRightAutonCtrl.updateCal();
		
		if(dtPGainCal.isChanged())
			dtPGainCal.acknowledgeValUpdate();
		if(dtIGainCal.isChanged())
			dtIGainCal.acknowledgeValUpdate();
		if(dtFGainCal.isChanged())
			dtFGainCal.acknowledgeValUpdate();
	}

	public void autonomousControl() {
		if(RobotState.visionAlignmentDesiried){
			if(RobotState.visionAlignmentPossible){
				//If we're in autonomous and vision alignment is desired (and possible), use the vision commands with no strafe
				runOpenLoop(RobotState.visionDtFwdRevCmd, 0, RobotState.visionDtRotateCmd, 0);
			} else {
				//If the auto routine wanted vision but we can't find a target, give up and stay still.
				runOpenLoop(0, 0, 0, 0);
			}

		} else {
			//If we're in autonomous but don't need vision alignment, use the PID commands
			 runClosedLoop(RobotState.autonDtfrontLeftWheelVelocityCmd_rpm,
					       RobotState.autonDtfrontRightWheelVelocityCmd_rpm,
					       RobotState.autonDtrearLeftWheelVelocityCmd_rpm,
					       RobotState.autonDtrearRightWheelVelocityCmd_rpm);
		}
		
		updateMotorCmds();
	}

	public void operatorControl() {
			
		if(RobotState.visionAlignmentDesiried & RobotState.visionAlignmentPossible){
			//For operator control, vision assist, get commands from the vision subsystem (although the driver may still strafe)
			runOpenLoop(RobotState.visionDtFwdRevCmd, RobotState.driverStrafeCmd, RobotState.visionDtRotateCmd, 0);
		} else if(fieldOrientedCtrl.get() == 0.0){
			//For operator control, non-field oriented, and no vision assist, get all commands from driver 
			runOpenLoop(RobotState.driverFwdRevCmd, RobotState.driverStrafeCmd, RobotState.driverRotateCmd, 0);
		} else {
			//For operator control, field oriented, and no vision assist, get all commands from driver along with gyro angle
			runOpenLoop(RobotState.driverFwdRevCmd, RobotState.driverStrafeCmd, RobotState.driverRotateCmd, RobotState.robotPoseAngle_deg);
		}
		
		updateMotorCmds();

	}
	
	/**
	 * Disables PI controllers (if needed) and sends commands to the open-loop drivetrain algorithm
	 * @param fwdRevCmd
	 * @param strafeCmd
	 * @param rotateCmd
	 * @param gyroAngle
	 */
	private void runOpenLoop(double fwdRevCmd, double strafeCmd, double rotateCmd, double gyroAngle){
		frontLeftAutonCtrl.setEnabled(false);
		frontRightAutonCtrl.setEnabled(false);
		rearLeftAutonCtrl.setEnabled(false);
		rearRightAutonCtrl.setEnabled(false);
		
		//Note this method inverts Y to match the joystick convention. But since
		//we already undid that joystick convention, we compensate here.
		myDrive.mecanumDrive_Cartesian(fwdRevCmd, -strafeCmd, rotateCmd, gyroAngle);
		
		runningClosedLoop = false;
	}
	
	/**
	 * Enables PI controllers (if not yet enabled) and updates their setpoints
	 * @param frontLeftSetpoint_RPM
	 * @param frontRightSetpoint_RPM
	 * @param rearLeftSetpoint_RPM
	 * @param rearRightSetpoint_RPM
	 */
	private void runClosedLoop(double frontLeftSetpoint_RPM, double frontRightSetpoint_RPM, double rearLeftSetpoint_RPM, double rearRightSetpoint_RPM){
		frontLeftAutonCtrl.setEnabled(true);
		frontRightAutonCtrl.setEnabled(true);
		rearLeftAutonCtrl.setEnabled(true);
		rearRightAutonCtrl.setEnabled(true);
		
		frontLeftAutonCtrl.setSetpoint(frontLeftSetpoint_RPM);
		frontRightAutonCtrl.setSetpoint(frontRightSetpoint_RPM);
		rearLeftAutonCtrl.setSetpoint(rearLeftSetpoint_RPM);
		rearRightAutonCtrl.setSetpoint(rearRightSetpoint_RPM);
		
		runningClosedLoop = true;
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
	
	public boolean isRunningClosedLoop(){
		return runningClosedLoop;
	}



}