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


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

import org.usfirst.frc.team1736.lib.Calibration.Calibration;

public class DriveTrain{
	
	private static DriveTrain driveTrain;
	
	private RobotDrive myDrive;

	private VictorSP frontLeftMotor;
	private VictorSP frontRightMotor;
	private VictorSP rearLeftMotor;
	private VictorSP rearRightMotor;
	
	private Encoder frontLeftEncoder;
	private Encoder frontRightEncoder;
	private Encoder rearLeftEncoder;
	private Encoder rearRightEncoder;
	
	private DriveTrainWheelSpeedPI frontLeftAutonCtrl;
	private DriveTrainWheelSpeedPI frontRightAutonCtrl;
	private DriveTrainWheelSpeedPI rearLeftAutonCtrl;
	private DriveTrainWheelSpeedPI rearRightAutonCtrl;
	
	Calibration dtPGainCal;
	Calibration dtFGainCal;
	Calibration dtIGainCal;
	
	Calibration fieldOrientedCtrl;
	
	private boolean runningClosedLoop;
	

	public static synchronized DriveTrain getInstance()
	{
		if(driveTrain == null)
			driveTrain = new DriveTrain();
		return driveTrain;
	}
	
	private DriveTrain() {
		//Setup Drivetrain with motors and such
		frontLeftMotor  = new RateLimitedVictorSP(RobotConstants.DRIVETRAIN_FRONT_LEFT_MOTOR);
		frontRightMotor = new RateLimitedVictorSP(RobotConstants.DRIVETRAIN_FRONT_RIGHT_MOTOR);
		rearLeftMotor   = new RateLimitedVictorSP(RobotConstants.DRIVETRAIN_REAR_LEFT_MOTOR);
		rearRightMotor  = new RateLimitedVictorSP(RobotConstants.DRIVETRAIN_REAR_RIGHT_MOTOR);
		
		//Set inversion on drivetrain motors (opposite sides need to be flipped in sign so positive command yeilds positive motion)
		frontLeftMotor.setInverted(false);
		frontRightMotor.setInverted(false);
		rearLeftMotor.setInverted(true);
		rearRightMotor.setInverted(true);
		
		
		myDrive = new RobotDrive(frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor);
		
		//Set up calibratable values
		fieldOrientedCtrl = new Calibration("Enable Field-Oriented Control", 0.0, 0.0, 1.0);
		dtPGainCal = new Calibration ("DT Auton Velocity P Gain", 0.004);
		dtFGainCal = new Calibration ("DT Auton Velocity F Gain", 0.00200);
		dtIGainCal = new Calibration ("DT Auton Velocity I Gain", 0.0004);
		
		//set up encoders
		frontLeftEncoder  = new Encoder(RobotConstants.DRIVETRAIN_FRONT_LEFT_ENCODER_A,  RobotConstants.DRIVETRAIN_FRONT_LEFT_ENCODER_B,  false);
		frontRightEncoder = new Encoder(RobotConstants.DRIVETRAIN_FRONT_RIGHT_ENCODER_A, RobotConstants.DRIVETRAIN_FRONT_RIGHT_ENCODER_B, false);
		rearLeftEncoder   = new Encoder(RobotConstants.DRIVETRAIN_REAR_LEFT_ENCODER_A,   RobotConstants.DRIVETRAIN_REAR_LEFT_ENCODER_B,   false);
		rearRightEncoder  = new Encoder(RobotConstants.DRIVETRAIN_REAR_RIGHT_ENCODER_A,  RobotConstants.DRIVETRAIN_REAR_RIGHT_ENCODER_B,  false);
		
		//Note minus signs to invert right side of drivetrain
		frontLeftEncoder.setDistancePerPulse(RobotConstants.DRIVETRAIN_WHEELS_REV_PER_TICK);
		frontRightEncoder.setDistancePerPulse(-RobotConstants.DRIVETRAIN_WHEELS_REV_PER_TICK);
		rearLeftEncoder.setDistancePerPulse(RobotConstants.DRIVETRAIN_WHEELS_REV_PER_TICK);
		rearRightEncoder.setDistancePerPulse(-RobotConstants.DRIVETRAIN_WHEELS_REV_PER_TICK);
		
		
		
		//Set up autonomous PI controllers
		frontLeftAutonCtrl = new DriveTrainWheelSpeedPI(frontLeftMotor,  frontLeftEncoder,  dtFGainCal, dtPGainCal, dtIGainCal);
		frontRightAutonCtrl = new DriveTrainWheelSpeedPI(frontRightMotor, frontRightEncoder, dtFGainCal, dtPGainCal, dtIGainCal);
		rearLeftAutonCtrl = new DriveTrainWheelSpeedPI(rearLeftMotor,   rearLeftEncoder,   dtFGainCal, dtPGainCal, dtIGainCal);
		rearRightAutonCtrl = new DriveTrainWheelSpeedPI(rearRightMotor,  rearRightEncoder,  dtFGainCal, dtPGainCal, dtIGainCal);
		
		//Invert controls on proper motors
		frontRightAutonCtrl.setOutputInverted(true);
		//frontRightAutonCtrl.setSensorInverted(true);
		rearLeftAutonCtrl.setOutputInverted(true);
		//rearLeftAutonCtrl.setSensorInverted(true);
		
		//Invert gyro comp where correct (experimentally determined)
		frontLeftAutonCtrl.setGyroCompInverted(true);
		frontRightAutonCtrl.setGyroCompInverted(true);
				
		runningClosedLoop = false;
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
		if(VisionAlignment.getInstance().getVisionAlignmentDesired()){
			if(VisionAlignment.getInstance().getVisionAlignmentPossible()){
				//If we're in autonomous and vision alignment is desired (and possible), use the vision commands with no strafe
				runOpenLoop(0, 0, VisionAlignment.getInstance().getRotateCmd(), 0);
			} else {
				//If the auto routine wanted vision but we can't find a target, give up and stay still.
				runOpenLoop(0, 0, 0, 0);
			}

		} else {
			//If we're in autonomous but don't need vision alignment, use the PID commands
			 runClosedLoop();
		}
	}

	public void operatorControl() {
		DriverController driverControl = DriverController.getInstance();
		if(VisionAlignment.getInstance().getVisionAlignmentDesired()){
			//For operator control, vision assist, get commands from the vision subsystem (although the driver may still strafe)
			runOpenLoop(driverControl.getFwdRevCmd(), driverControl.getStrafeCmd(), VisionAlignment.getInstance().getRotateCmd(), 0);
		} else if(fieldOrientedCtrl.get() == 0.0){
			//For operator control, non-field oriented, and no vision assist, get all commands from driver 
			runOpenLoop(driverControl.getFwdRevCmd(), driverControl.getStrafeCmd(), driverControl.getRotateCmd(), 0);
			
			/*
			//Tuning temp
			frontLeftAutonCtrl.setSetpoint(600*Math.sin(Timer.getFPGATimestamp()*2*Math.PI*0.175));
			frontRightAutonCtrl.setSetpoint(600*Math.sin(Timer.getFPGATimestamp()*2*Math.PI*0.175));
			rearLeftAutonCtrl.setSetpoint(600*Math.sin(Timer.getFPGATimestamp()*2*Math.PI*0.175));
			rearRightAutonCtrl.setSetpoint(600*Math.sin(Timer.getFPGATimestamp()*2*Math.PI*0.175));
			runClosedLoop();
			*/
		} else {
			//For operator control, field oriented, and no vision assist, get all commands from driver along with gyro angle
			runOpenLoop(driverControl.getFwdRevCmd(), driverControl.getStrafeCmd(), driverControl.getRotateCmd(), Gyro.getInstance().getAngle());
		}

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
	 * Enables PI controllers (if not yet enabled).  Assumes setpoints for PI controllers are managed elsewhere
	 */
	private void runClosedLoop(){
		frontLeftAutonCtrl.setEnabled(true);
		frontRightAutonCtrl.setEnabled(true);
		rearLeftAutonCtrl.setEnabled(true);
		rearRightAutonCtrl.setEnabled(true);
		
		runningClosedLoop = true;
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
	
	public DriveTrainWheelSpeedPI getFrontLeftCTRL(){
		return frontLeftAutonCtrl;
	}
	
	public DriveTrainWheelSpeedPI getFrontRightCTRL(){
		return frontRightAutonCtrl;
	}

	public DriveTrainWheelSpeedPI getRearLeftCTRL(){
		return rearLeftAutonCtrl;
	}
	
	public DriveTrainWheelSpeedPI getRearRightCTRL(){
		return rearRightAutonCtrl;
	}
	
	public double getFrontLeftWheelSpeedRPM()
	{
		return frontLeftEncoder.getRate()*60.0;
	}
	
	public double getFrontRightWheelSpeedRPM()
	{
		return frontRightEncoder.getRate()*60.0;
	}
	
	public double getRearLeftWheelSpeedRPM()
	{
		return rearLeftEncoder.getRate()*60.0;
	}
	
	public double getRearRightWheelSpeedRPM()
	{
		return rearRightEncoder.getRate()*60.0;
	}
	
	public double getHeadingSetpoint(){
		return frontLeftAutonCtrl.getDesiredHeading();
	}
	
	public double getFrontLeftDesiredWheelSpeedRPM()
	{
		return frontLeftAutonCtrl.getSetpoint();
	}
	
	public double getFrontRightDesiredWheelSpeedRPM()
	{
		return frontRightAutonCtrl.getSetpoint();
	}
	
	public double getRearLeftDesiredWheelSpeedRPM()
	{
		return rearLeftAutonCtrl.getSetpoint();
	}
	
	public double getRearRightDesiredWheelSpeedRPM()
	{
		return rearRightAutonCtrl.getSetpoint();
	}
	
	public double getFrontLeftWheelDistanceFt()
	{
		return frontLeftEncoder.getDistance()*2.0*Math.PI*RobotConstants.DRIVETRAIN_WHEELS_RADIUS_FT;
	}
	
	public double getFrontRightWheelDistanceFt()
	{
		return frontRightEncoder.getDistance()*2.0*Math.PI*RobotConstants.DRIVETRAIN_WHEELS_RADIUS_FT;
	}
	
	public double getRearLeftWheelDistanceFt()
	{
		return rearLeftEncoder.getDistance()*2.0*Math.PI*RobotConstants.DRIVETRAIN_WHEELS_RADIUS_FT;
	}
	
	public double getRearRightWheelDistanceFt()
	{
		return rearRightEncoder.getDistance()*2.0*Math.PI*RobotConstants.DRIVETRAIN_WHEELS_RADIUS_FT;
	}
	
	public void disableSafety()
	{
		myDrive.setSafetyEnabled(false);
	}
	
	public static double FtPerSec_to_RPM(double ft_per_sec_in){
		return ft_per_sec_in * 60.0 / (2*Math.PI*RobotConstants.DRIVETRAIN_WHEELS_RADIUS_FT);
	}
}