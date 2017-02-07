package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
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
    	dtPGainCal = new Calibration ("DT Auton Velocity P Gain", 0.0);
    	dtFGainCal = new Calibration ("DT Auton Velocity F Gain", 0.0005);
    	dtIGainCal = new Calibration ("DT Auton Velocity I Gain", 0.0);
    	
    	//set up encoders
    	frontLeftEncoder  = new Encoder(RobotConstants.DRIVETRAIN_FRONT_LEFT_ENCODER_A,  RobotConstants.DRIVETRAIN_FRONT_LEFT_ENCODER_B,  false);
    	frontRightEncoder = new Encoder(RobotConstants.DRIVETRAIN_FRONT_RIGHT_ENCODER_A, RobotConstants.DRIVETRAIN_FRONT_RIGHT_ENCODER_B, false);
    	rearLeftEncoder   = new Encoder(RobotConstants.DRIVETRAIN_REAR_LEFT_ENCODER_A,   RobotConstants.DRIVETRAIN_REAR_LEFT_ENCODER_B,   false);
    	rearRightEncoder  = new Encoder(RobotConstants.DRIVETRAIN_REAR_RIGHT_ENCODER_A,  RobotConstants.DRIVETRAIN_REAR_RIGHT_ENCODER_B,  false);
    	
    	//Note minus signs to invert right side of drivetrain
    	frontLeftEncoder.setDistancePerPulse(RobotConstants.DRIVETRAIN_WHEELS_REV_PER_TICK);
    	frontRightEncoder.setDistancePerPulse(-RobotConstants.DRIVETRAIN_WHEELS_REV_PER_TICK);
    	rearLeftEncoder.setDistancePerPulse(-RobotConstants.DRIVETRAIN_WHEELS_REV_PER_TICK);
    	rearRightEncoder.setDistancePerPulse(RobotConstants.DRIVETRAIN_WHEELS_REV_PER_TICK);
    	
    	
    	
    	//Set up autonomous PI controllers
    	frontLeftAutonCtrl = new DriveTrainWheelSpeedPI(frontLeftMotor,  frontLeftEncoder,  dtFGainCal, dtPGainCal, dtIGainCal);
    	frontRightAutonCtrl = new DriveTrainWheelSpeedPI(frontRightMotor, frontRightEncoder, dtFGainCal, dtPGainCal, dtIGainCal);
    	rearLeftAutonCtrl = new DriveTrainWheelSpeedPI(rearLeftMotor,   rearLeftEncoder,   dtFGainCal, dtPGainCal, dtIGainCal);
    	rearRightAutonCtrl = new DriveTrainWheelSpeedPI(rearRightMotor,  rearRightEncoder,  dtFGainCal, dtPGainCal, dtIGainCal);
    	
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
				runOpenLoop(VisionAlignment.getInstance().getFwdRevCmd(), 0, VisionAlignment.getInstance().getRotateCmd(), 0);
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
			runOpenLoop(VisionAlignment.getInstance().getFwdRevCmd(), driverControl.getStrafeCmd(), VisionAlignment.getInstance().getRotateCmd(), 0);
		} else if(fieldOrientedCtrl.get() == 0.0){
			//For operator control, non-field oriented, and no vision assist, get all commands from driver 
			runOpenLoop(driverControl.getFwdRevCmd(), driverControl.getStrafeCmd(), driverControl.getRotateCmd(), 0);
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
}