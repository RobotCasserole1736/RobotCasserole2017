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

import org.usfirst.frc.team1736.lib.Calibration.CalWrangler;
import org.usfirst.frc.team1736.lib.LoadMon.CasseroleRIOLoadMonitor;
import org.usfirst.frc.team1736.lib.Logging.CsvLogger;
import org.usfirst.frc.team1736.lib.WebServer.CasseroleDriverView;
import org.usfirst.frc.team1736.lib.WebServer.CasseroleWebServer;
import org.usfirst.frc.team1736.lib.WebServer.CassesroleWebStates;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	
	///////////////////////////////////////////////////////////////////
	// Robot Class-Scope Objects
	///////////////////////////////////////////////////////////////////
	
	//RIO Performance Monitoring
	private double prevLoopStartTimestamp;
	double loopTimeElapsed;
	CasseroleRIOLoadMonitor ecuStats;
	
	// Physical Devices on the robot
	PowerDistributionPanel pdp;
	LowBatteryIndicator lowBatt;
	
	// Air pressure
	double currAirPress;
	
	Gyro gyro;
	DriveTrain driveTrain;

	//Vision Processing Algorithm
	VisionProcessing visionProc;
	
	//Vision system delay measurement & Calibration
	VisionDelayCalibration visionDelayCal;
	
	//Software utilities
	RobotPoseCalculator poseCalc;
	CalWrangler wrangler;
	CasseroleWebServer webServer;

	//Controllers
	DriverController driverCTRL;
	OperatorController operatorCTRL;

	//Hopper Feed Control
	HopperControl hopControl; 
	
	//Intake Control
	IntakeControl intakeControl;
	
	//Shooter wheel control
	ShooterWheelCtrl shooterWheelControl;
	
	//Climber Control
	ClimberControl climbControl;

	//Camera gimbal mount
	CameraServoMount camGimbal;	

	//Operator shooter command interpretation variables
	boolean pev_State;
	
	//Shooter control
	ShotControl shotCTRL;
	
	//Shot Count
	ShotCounter shotCount;

	//Vision Alignment Control
	VisionAlignment visionAlignCTRL;
	
	boolean autoAlignNotPossibleDVIndState;
	
	//Gear control subsystem (kinda mashed in here just cuz we're lazy)
	Solenoid gearSolenoid;
	
	
	//LED's 
	LEDSequencer LEDseq;
	
	//Compressor & Sensor system
	PneumaticsSupply airCompressor;

	//Autonomous Routines
	Autonomous auto;
	
	//Driver Station (info about the match)
	DriverStation ds;
	
	///////////////////////////////////////////////////////////////////
	// Robot Top-Level Methods
	///////////////////////////////////////////////////////////////////
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit(){
		//Set up physical devices
		driveTrain = DriveTrain.getInstance();
		pdp = new PowerDistributionPanel();
		lowBatt = LowBatteryIndicator.getInstance();
		lowBatt.setPDPReference(pdp);
		gyro = Gyro.getInstance();
		visionProc = VisionProcessing.getInstance();
		visionAlignCTRL = VisionAlignment.getInstance();
		visionDelayCal = VisionDelayCalibration.getInstance();

		ecuStats = new CasseroleRIOLoadMonitor();
		poseCalc = RobotPoseCalculator.getInstance();
		shotCTRL = ShotControl.getInstance();
		shotCount = ShotCounter.getInstance();
		hopControl = HopperControl.getInstance();
		shooterWheelControl = ShooterWheelCtrl.getInstance();
		climbControl = ClimberControl.getInstance();
		intakeControl = IntakeControl.getInstance();
		airCompressor = new PneumaticsSupply();

		camGimbal = new CameraServoMount();
		
		gearSolenoid = new Solenoid(RobotConstants.GEAR_SOLENOID_PORT);

		auto = new Autonomous(driveTrain);

		

		driverCTRL = DriverController.getInstance();
		operatorCTRL = OperatorController.getInstance();
		driverCTRL.setDeadzone(0.175);
		operatorCTRL.setDeadzone(0.175);
		
		LEDseq = new LEDSequencer();
		
		autoAlignNotPossibleDVIndState = false;
		
		ds = DriverStation.getInstance();

		initLoggingChannels();
		
		initDriverView();
		
		//Set up and start web server (must be after all other website init functions)
		webServer = new CasseroleWebServer();
		webServer.startServer();
		
		//Load any saved calibration values (must be last to ensure all calibrations have been initialized first)
		CalWrangler.loadCalValues();
		
	}
	
	/**
	 * This function is called just before the robot enters disabled
	 */
	@Override
	public void disabledInit() {
		
		//Close out any log which is running
		CsvLogger.close();
		autoAlignNotPossibleDVIndState = false;
	}
	
	/**
	 * This function is called periodically while disabled. Note no motors should be commanded on while in this state.
	 */
	@Override
	public void disabledPeriodic() {
		
		//Mark start of loop, Initialize Timer
		//Must be as close to the start of the loop as possible
		prevLoopStartTimestamp = Timer.getFPGATimestamp();
		
		//Get all inputs from outside the robot
		poseCalc.update();
		
		//Update vision processing algorithm to find any targets on in view
		visionProc.update();
		
		//Update any calibration which is running
		visionDelayCal.update();
		
		//Update select PID gains from calibrations (only do during disabled to prevent potential gain-switching instability)
		shooterWheelControl.updateGains();
		driveTrain.updateAllCals();
		visionAlignCTRL.updateGains();
		
		LEDseq.setDisabledPattern();
		
		updateDriverView();
		updateWebStates();

		//Mark end of loop and Calculate Loop Time
		//Must be as close to the end of the loop as possible.
		loopTimeElapsed = Timer.getFPGATimestamp() - prevLoopStartTimestamp;
		
	}
		
		
		
		
		
		
		
		
	///////////////////////////////////////////////////////////////////
	// Autonomous top-level Methods
	///////////////////////////////////////////////////////////////////
	/**
	 * This method gets run right before the robot starts running autonomous mode.
	 */
	@Override
	public void autonomousInit() {	
		driveTrain.disableSafety();
		
		loopTimeElapsed = 0;

		//Assume starting at 0 degrees
		gyro.reset();
		
		//Presume autonomous starts at zero distance (robot initial orientation is origin)
		driveTrain.resetAllEncoders();
		driveTrain.resetAllIntegrators();
		
		//Open a new log
		CsvLogger.init();
		
		auto.executeAutonomus();

	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		
		//Mark start of loop, Initialize Timer
		//Must be as close to the start of the loop as possible
		prevLoopStartTimestamp = Timer.getFPGATimestamp();
		
		//Get all inputs from outside the robot
		poseCalc.update();
		
		//Update vision processing algorithm to find any targets on in view
		visionProc.update();
		
		//Run vision alignment algorithm
		visionAlignCTRL.GetAligned();
		
		//Update any calibration which is running
		visionDelayCal.update();
		
		//Update shot control management subsystem
		shotCTRL.update();
		
		//Update Shot Counter
		shotCount.update();
		
		//Update Hopper Control
		hopControl.update();
		
		//Update Intake Control
		intakeControl.update();
		
		//Update shooter wheel control
		shooterWheelControl.update();
		
		//Run drivetrain in autonomous
		driveTrain.autonomousControl();
		
		//Update Climber Control 
		climbControl.update();
		
		//Update the low battery parameter checker
		lowBatt.update();
		
		auto.update();
		
		LEDseq.setAutonPattern();
		
		//Log & display present state data
		updateDriverView();
		CsvLogger.logData(false);
		updateWebStates();

		//Mark end of loop and Calculate Loop Time
		//Must be as close to the end of the loop as possible.
		loopTimeElapsed = Timer.getFPGATimestamp() - prevLoopStartTimestamp;
	}
	
	
	
	
	
	
	
	///////////////////////////////////////////////////////////////////
	// Teleoperated top-level Methods
	///////////////////////////////////////////////////////////////////
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopInit() {
		driveTrain.disableSafety();

		auto.stop();
		loopTimeElapsed = 0;
		
		driveTrain.resetAllIntegrators();
		
		//Open a new log
		CsvLogger.init();
	}	
	
	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		//Initialize Timer
		prevLoopStartTimestamp = Timer.getFPGATimestamp();
		
		//Get all inputs from outside the robot
		updateAllHumanInputs();
		poseCalc.update();
		
		
		//Update vision processing algorithm to find any targets on in view
		visionProc.update();

		//Run vision alignment algorithm
		visionAlignCTRL.setVisionAlignmentDesired(DriverController.getInstance().getAlignDesired());
		visionAlignCTRL.GetAligned();
		
		//Update any calibration which is running
		visionDelayCal.update();
		
		//Update shot control management subsystem
		shotCTRL.update();
		
		//Update shot counter
		shotCount.update();
		
		//Update Hopper Control
		hopControl.update();
		
		//Update Intake Control
		intakeControl.update();
		
		//Update shooter wheel control
		shooterWheelControl.update();
		
		//Run drivetrain in operator control
		driveTrain.operatorControl();
		
		//Update Climber Control 
		climbControl.update();
		
		//Update low battery parameter checker
		lowBatt.update();
		

		//Update user camera
		camGimbal.update();
		
		
		//Log & display present state data
		updateDriverView();
		CsvLogger.logData(false);
		updateWebStates();
		//Mark end of loop and Calculate Loop Time
		//Must be as close to the end of the loop as possible.
		loopTimeElapsed = Timer.getFPGATimestamp() - prevLoopStartTimestamp;
	}
	
	
	
	
	///////////////////////////////////////////////////////////////////
	// Utility MethodsS
	///////////////////////////////////////////////////////////////////
	
	//Sets up all the logged channels of data. Should be called once before opening any logs
	public void initLoggingChannels(){
		CsvLogger.addLoggingFieldDouble("TIME","sec","getFPGATimestamp",Timer.class);
		CsvLogger.addLoggingFieldDouble("Match_Time","sec","getMatchTime",ds);
		CsvLogger.addLoggingFieldBoolean("DriverStation_Attached","bit","isDSAttached",ds);
		CsvLogger.addLoggingFieldBoolean("FPGA_Active","bit","isSysActive",ds);
		CsvLogger.addLoggingFieldBoolean("Brownout_Protection_Active","bit","isBrownedOut",ds);
		CsvLogger.addLoggingFieldDouble("PDP_Voltage","V","getVoltage", pdp);
		CsvLogger.addLoggingFieldDouble("PDP_Total_Current","A","getTotalCurrent", pdp);
		CsvLogger.addLoggingFieldDouble("PDP_DT_FL_Current","A","getCurrent", pdp, RobotConstants.DRIVETRAIN_FRONT_LEFT_PDP_CH);
		CsvLogger.addLoggingFieldDouble("PDP_DT_FR_Current","A","getCurrent", pdp, RobotConstants.DRIVETRAIN_FRONT_RIGHT_PDP_CH);
		CsvLogger.addLoggingFieldDouble("PDP_DT_RL_Current","A","getCurrent", pdp, RobotConstants.DRIVETRAIN_REAR_LEFT_PDP_CH);
		CsvLogger.addLoggingFieldDouble("PDP_DT_RR_Current","A","getCurrent", pdp, RobotConstants.DRIVETRAIN_REAR_RIGHT_PDP_CH);
		CsvLogger.addLoggingFieldDouble("Hopper_Motor_Current","A","getCurrent", pdp,  RobotConstants.HOPPER_MOTOR_PDP_CH);
		CsvLogger.addLoggingFieldDouble("Climber_Motor1_Current","A","getCurrent", pdp, RobotConstants.CLIMBER_MOTOR1_PDP_CH);
		CsvLogger.addLoggingFieldDouble("Climber_Motor2_Current","A","getCurrent", pdp, RobotConstants.CLIMBER_MOTOR2_PDP_CH);
		CsvLogger.addLoggingFieldDouble("Intake_Motor_Current","A","getCurrent", pdp,  RobotConstants.INTAKE_MOTOR_PDP_CH);
		CsvLogger.addLoggingFieldDouble("Shooter_Motor_Current","A","getOutputCurrent",shooterWheelControl);
		CsvLogger.addLoggingFieldDouble("RIO_Loop_Time","msec","getLoopTime_ms", this);
		CsvLogger.addLoggingFieldDouble("RIO_Cpu_Load","%","getCpuLoad", this);
		CsvLogger.addLoggingFieldDouble("RIO_RAM_Usage","%","getRAMUsage", this);
		CsvLogger.addLoggingFieldDouble("Driver_FwdRev_cmd","cmd","getFwdRevCmd", driverCTRL);
		CsvLogger.addLoggingFieldDouble("Driver_Strafe_cmd","cmd","getStrafeCmd", driverCTRL);
		CsvLogger.addLoggingFieldDouble("Driver_Rotate_cmd","cmd","getRotateCmd", driverCTRL);
		CsvLogger.addLoggingFieldBoolean("Driver_Vision_Align_Desired","bit","getVisionAlignmentDesired", visionAlignCTRL);
//		CsvLogger.addLoggingFieldDouble("Auton_DT_FL_Desired_Velocity","RPM","getAutonDtfrontLeftWheelVelocityCmd_rpm", RobotState.class);
//		CsvLogger.addLoggingFieldDouble("Auton_DT_FR_Desired_Velocity","RPM","getAutonDtfrontRightWheelVelocityCmd_rpm", RobotState.class);
//		CsvLogger.addLoggingFieldDouble("Auton_DT_RL_Desired_Velocity","RPM","getAutonDtrearLeftWheelVelocityCmd_rpm", RobotState.class);
//		CsvLogger.addLoggingFieldDouble("Auton_DT_RR_Desired_Velocity","RPM","getAutonDtrearRightWheelVelocityCmd_rpm", RobotState.class);
		CsvLogger.addLoggingFieldBoolean("DT_Running_Closed_Loop","bit","isRunningClosedLoop",driveTrain);
		CsvLogger.addLoggingFieldDouble("Robot_FwdRev_Vel","ft/sec","getFwdRevVelFtPerS", poseCalc);
		CsvLogger.addLoggingFieldDouble("Robot_Strafe_Vel","ft/sec","getStrafeVelFtPerS", poseCalc);
		CsvLogger.addLoggingFieldDouble("Robot_FwdRev_Dist","ft","getFwdRevDistFt", poseCalc);
		CsvLogger.addLoggingFieldDouble("Robot_Strafe_Dist","ft","getStrafeDistFt", poseCalc);
		CsvLogger.addLoggingFieldDouble("Robot_Pose_Angle","deg","getAngle", gyro);
		CsvLogger.addLoggingFieldBoolean("Op_Gear_Release_Desired","bit","getGearSolenoidCmd", operatorCTRL);
		CsvLogger.addLoggingFieldDouble("Shot_State_Command","bits","getDesiredShooterStateOrdinal", shotCTRL);
		CsvLogger.addLoggingFieldDouble("Hopper_Feed_Cmd","cmd","getHopperMotorCmd", hopControl);
		CsvLogger.addLoggingFieldDouble("Intake_Speed_Cmd","cmd","getCommandedIntakeSpeed", intakeControl);
		CsvLogger.addLoggingFieldBoolean("Op_Intake_Desired","bit","getIntakeDesiredCmd", operatorCTRL);
		CsvLogger.addLoggingFieldBoolean("Op_Eject_Desired","bit","getEjectDesiredCmd",operatorCTRL);
		CsvLogger.addLoggingFieldDouble("Climb_Speed_Cmd","cmd","getClimbSpeedCmd", operatorCTRL);
		CsvLogger.addLoggingFieldDouble("Shooter_Desired_Velocity","rpm","getShooterDesiredRPM", shooterWheelControl);
		CsvLogger.addLoggingFieldDouble("Shooter_Actual_Velocity","rpm","getShooterActualVelocityRPM", shooterWheelControl);
		CsvLogger.addLoggingFieldDouble("Shooter_Motor_Cmd","rpm","getShooterMotorCmd", shooterWheelControl);
		CsvLogger.addLoggingFieldBoolean("Shooter_Velocity_OK","bit","getShooterVelocityOK", shooterWheelControl);
		CsvLogger.addLoggingFieldDouble("Shot_Counter", "count", "getCurrCountLog", shotCount);
		CsvLogger.addLoggingFieldDouble("FL_Motor_Cmd","cmd","getFLDriveMotorCmd", driveTrain);
		CsvLogger.addLoggingFieldDouble("FR_Motor_Cmd","cmd","getFRDriveMotorCmd", driveTrain);
		CsvLogger.addLoggingFieldDouble("RL_Motor_Cmd","cmd","getRLDriveMotorCmd", driveTrain);
		CsvLogger.addLoggingFieldDouble("RR_Motor_Cmd","cmd","getRRDriveMotorCmd", driveTrain);
		CsvLogger.addLoggingFieldDouble("FL_Wheel_Velocity","rpm","getFrontLeftWheelSpeedRPM", driveTrain);
		CsvLogger.addLoggingFieldDouble("FR_Wheel_Velocity","rpm","getFrontRightWheelSpeedRPM", driveTrain);
		CsvLogger.addLoggingFieldDouble("RL_Wheel_Velocity","rpm","getRearLeftWheelSpeedRPM", driveTrain);
		CsvLogger.addLoggingFieldDouble("RR_Wheel_Velocity","rpm","getRearRightWheelSpeedRPM", driveTrain);
		CsvLogger.addLoggingFieldBoolean("Vision_System_Online","bit","isOnline", visionProc);
		CsvLogger.addLoggingFieldBoolean("Valid_Vision_Target_Found","bit","isTargetFound", visionProc.getTarget());
		CsvLogger.addLoggingFieldDouble("Vision_Target_Angle_From_Camera","deg","getTargetOffsetDegrees", visionProc.getTarget());
		CsvLogger.addLoggingFieldDouble("Vision_Target_Gyro_Actual_Angle_At_Frame","deg","getGyroAngleAtLastFrame", visionAlignCTRL);
		CsvLogger.addLoggingFieldDouble("Vision_Target_Gyro_Desired_Angle_At_Frame","deg","getGyroAngleDesiredAtLastFrame", visionAlignCTRL);
		CsvLogger.addLoggingFieldDouble("Vision_Target_Range","ft","getEstTargetDistanceFt", visionProc.getTarget());
		CsvLogger.addLoggingFieldDouble("Vision_Process_Time","msec","getVisionProcessTimeMs",visionProc);
		CsvLogger.addLoggingFieldDouble("Vision_CoProc_FPS","frames/sec","getCoProcessorFPS", visionProc);
		CsvLogger.addLoggingFieldDouble("Vision_CoProc_CPU_load","%","getCoProcessorCPULoadPct", visionProc);
		CsvLogger.addLoggingFieldDouble("Vision_CoProc_Mem_load","%","getCoProcessorMemLoadPct", visionProc);
		CsvLogger.addLoggingFieldBoolean("Vision_Align_Possible","bit","getVisionAlignmentPossible", visionAlignCTRL);
		CsvLogger.addLoggingFieldDouble("Vision_DT_FwdRev_Cmd","cmd","getFwdRevCmd", visionAlignCTRL);
		CsvLogger.addLoggingFieldDouble("Vision_DT_Rotate_Cmd","cmd","getRotateCmd", visionAlignCTRL);
		CsvLogger.addLoggingFieldBoolean("Vision_Align_On_Target","cmd","getVisionAlignmentOnTarget", visionAlignCTRL);
		CsvLogger.addLoggingFieldDouble("Vision_Align_State", "states", "getVisionAlignState", visionAlignCTRL);
		CsvLogger.addLoggingFieldDouble("Air_Pressure", "psi", "getPress", airCompressor);
		CsvLogger.addLoggingFieldDouble("Compressor_Current", "A", "getCompCurrent", airCompressor);
		CsvLogger.addLoggingFieldBoolean("Gear_Solenoid_Cmd","bit","get", gearSolenoid);
		CsvLogger.addLoggingFieldBoolean("Battery_Dead", "bit", "isBatteryDead", lowBatt);
		CsvLogger.preCacheAllMethods();
	}
	
	
	public void initDriverView(){
		CasseroleDriverView.newDial("RobotSpeed ft/sec", 0, 25, 5, 0, 20);
		CasseroleDriverView.newDial("Shooter Speed RPM", 0, 5000, 500, shotCTRL.wheel_Set_Point_rpm.get() - shooterWheelControl.ErrorRange.get(), 
																	   shotCTRL.wheel_Set_Point_rpm.get() + shooterWheelControl.ErrorRange.get());
		CasseroleDriverView.newDial("AirPressure Psi", 0, 130, 10, 100, 120);
		
		CasseroleDriverView.newBoolean("Vision Offline", "red");
		CasseroleDriverView.newBoolean("Target in View", "green");
		CasseroleDriverView.newBoolean("Vision Aligning", "yellow");
		CasseroleDriverView.newBoolean("Shooter Spoolup", "yellow");
		CasseroleDriverView.newBoolean("System Pressure Low", "red");
		CasseroleDriverView.newBoolean("Cmprsr Disabled", "yellow");
		CasseroleDriverView.newBoolean("Gyro Offline", "red");
		CasseroleDriverView.newBoolean("Low Battery", "yellow");
		CasseroleDriverView.newBoolean("AutoAlign Not Possible!", "red");
		
		CasseroleDriverView.newStringBox("Shot_Count");
		CasseroleDriverView.newStringBox("Orientation Deg");
		CasseroleDriverView.newStringBox("Vision Range Ft");
		CasseroleDriverView.newStringBox("Vision Angle Deg");
		CasseroleDriverView.newStringBox("Auton. Routine");
		CasseroleDriverView.newWebcam("VisionProc_cam", RobotConstants.VISION_PROC_CAMERA_URL, 50, 50, 0);
		CasseroleDriverView.newWebcam("Driver_cam", RobotConstants.DRIVER_CAMERA_URL, 50, 50, 0);

	}
	
	public void updateDriverView(){
		CasseroleDriverView.setDialValue("RobotSpeed ft/sec", poseCalc.getNetSpeedFtPerS());
		CasseroleDriverView.setDialValue("Shooter Speed RPM", shooterWheelControl.getShooterActualVelocityRPM());
		CasseroleDriverView.setDialValue("AirPressure Psi", airCompressor.getPress());
		CasseroleDriverView.setBoolean("Vision Offline", !visionProc.isOnline());
		CasseroleDriverView.setBoolean("Target in View", visionProc.getTarget().isTargetFound() && visionProc.isOnline());
		CasseroleDriverView.setBoolean("Vision Aligning", visionAlignCTRL.getVisionAlignState() == 1.0);
		CasseroleDriverView.setBoolean("Shooter Spoolup", (shooterWheelControl.getShooterDesiredRPM() > 100) && !(shooterWheelControl.getShooterVelocityOK()));
		CasseroleDriverView.setBoolean("System Pressure Low", (airCompressor.getPress() < RobotConstants.SYS_AIR_PRESSURE_CRITICAL_THRESH_PSI));
		CasseroleDriverView.setBoolean("Cmprsr Disabled", !airCompressor.isEnabled());
		CasseroleDriverView.setBoolean("Gyro Offline", !gyro.isOnline());
		CasseroleDriverView.setBoolean("Low Battery", lowBatt.isBatteryDead());
		CasseroleDriverView.setBoolean("AutoAlign Not Possible!", autoAlignNotPossibleDVIndState);
		
		CasseroleDriverView.setStringBox("Shot_Count", leftJustifyDouble(shotCount.getCurrCountLog()));
		CasseroleDriverView.setStringBox("Orientation Deg", leftJustifyDouble(gyro.getAngle() % 360.0));
		CasseroleDriverView.setStringBox("Vision Range Ft", leftJustifyDouble(visionProc.getTarget().getEstTargetDistanceFt()));
		CasseroleDriverView.setStringBox("Vision Angle Deg", leftJustifyDouble(visionProc.getTarget().getTargetOffsetDegrees()));
		CasseroleDriverView.setStringBox("Auton. Routine", "None"); // temp
		CasseroleDriverView.setWebcamCrosshairs("VisionProc_cam", 
												(visionProc.getTarget().getTopTargetXPixelPos()/RobotConstants.VISION_X_PIXELS) * 100.0, 
												(visionProc.getTarget().getTopTargetYPixelPos()/RobotConstants.VISION_Y_PIXELS) * 100.0);
	}
	
	private String leftJustifyDouble(double input){
		String temp = String.format("%.1f", input);
		for(int ii = 0; ii < 5 - temp.length(); ii++){
			temp = " " + temp; 
		}
		return temp;
	}
	
	//Puts all relevant data to the robot State webpage
	public void updateWebStates(){
		CassesroleWebStates.putDouble("Loop Time (ms)",	getLoopTime_ms());
		CassesroleWebStates.putDouble("CPU Load (%)",	  getCpuLoad()); 
		CassesroleWebStates.putDouble("RAM Usage (%)",	 getRAMUsage()); 
		CassesroleWebStates.putDouble("Driver FwdRev Cmd", driverCTRL.getFwdRevCmd());
		CassesroleWebStates.putDouble("Driver Strafe Cmd", driverCTRL.getStrafeCmd());
		CassesroleWebStates.putDouble("Driver Rotate Cmd", driverCTRL.getRotateCmd());
		CassesroleWebStates.putString("Op Shot Command", shotCTRL.getDesiredShooterState().toString());
		CassesroleWebStates.putDouble("Shooter Wheel Command", shooterWheelControl.getShooterMotorCmd());
		CassesroleWebStates.putDouble("Shooter Desired Speed (RPM)", shooterWheelControl.getShooterDesiredRPM());
		CassesroleWebStates.putDouble("Shooter Actual Speed (RPM)", shooterWheelControl.getShooterActualVelocityRPM());
		CassesroleWebStates.putBoolean("Shooter Speed OK", shooterWheelControl.getShooterVelocityOK());
		CassesroleWebStates.putDouble("Shot_Count", shotCount.getCurrCountLog());
		CassesroleWebStates.putDouble("Hopper Feed Cmd",   hopControl.getHopperMotorCmd());
		CassesroleWebStates.putBoolean("Gear Release Solenoid Cmd", gearSolenoid.get());
		CassesroleWebStates.putDouble("Intake Speed Cmd",   intakeControl.getCommandedIntakeSpeed());
		CassesroleWebStates.putDouble("Climb Speed Cmd",   operatorCTRL.getClimbSpeedCmd());
		CassesroleWebStates.putDouble("Robot FwdRev Velocity (ft per sec)",   poseCalc.getFwdRevVelFtPerS());
		CassesroleWebStates.putDouble("Robot Strafe Velocity (ft per sec)",   poseCalc.getStrafeVelFtPerS());
		CassesroleWebStates.putDouble("Robot FwdRev Distance (ft)",   poseCalc.getFwdRevDistFt());
		CassesroleWebStates.putDouble("Robot Strafe Distance (ft)",   poseCalc.getStrafeDistFt());
		CassesroleWebStates.putDouble("Robot Yaw (deg)",   gyro.getAngle());
		CassesroleWebStates.putDouble("Front Left Motor Output",   driveTrain.getFLDriveMotorCmd());
		CassesroleWebStates.putDouble("Front Right Motor Output",   driveTrain.getFRDriveMotorCmd());
		CassesroleWebStates.putDouble("Rear Left Motor Output",   driveTrain.getRLDriveMotorCmd());
		CassesroleWebStates.putDouble("Rear Right Motor Output",   driveTrain.getRRDriveMotorCmd());
		CassesroleWebStates.putDouble("Front Left Motor Speed (RPM)",   driveTrain.getFrontLeftWheelSpeedRPM());
		CassesroleWebStates.putDouble("Front Right Motor Speed (RPM)",   driveTrain.getFrontRightWheelSpeedRPM());
		CassesroleWebStates.putDouble("Rear Left Motor Speed (RPM)",   driveTrain.getRearLeftWheelSpeedRPM());
		CassesroleWebStates.putDouble("Rear Right Motor Speed (RPM)",   driveTrain.getRearRightWheelSpeedRPM());
		CassesroleWebStates.putBoolean("Vision CoProcessor Online", visionProc.isOnline());
		CassesroleWebStates.putDouble("Vision CoProcessor FPS", visionProc.getCoProcessorFPS());
		CassesroleWebStates.putDouble("Vision CoProcessor CPU Load (%)", visionProc.getCoProcessorCPULoadPct());
		CassesroleWebStates.putDouble("Vision CoProcessor Mem Load (%)", visionProc.getCoProcessorMemLoadPct());
		CassesroleWebStates.putDouble("Vision Num Contours Observed", visionProc.getNumberOfTargetsObserved());
		CassesroleWebStates.putBoolean("Vision Target Seen", visionProc.getTarget().isTargetFound());
		CassesroleWebStates.putDouble("Vision Target Pixel Pos X", visionProc.getTarget().getTopTargetXPixelPos());
		CassesroleWebStates.putDouble("Vision Target Pixel Pos Y", visionProc.getTarget().getTopTargetYPixelPos());
		CassesroleWebStates.putDouble("Vision Target Range (ft)", visionProc.getTarget().getEstTargetDistanceFt());
		CassesroleWebStates.putDouble("Vision Target Offset (deg)", visionProc.getTarget().getTargetOffsetDegrees());
		CassesroleWebStates.putDouble("Vision Heuristic Val", visionProc.getCurHeuristic());
		CassesroleWebStates.putDouble("Vision Proc Delay (ms)", (Timer.getFPGATimestamp() - visionProc.getEstCaptureTime())*1000);
		CassesroleWebStates.putDouble("Vision Fwd/Rev Cmd", visionAlignCTRL.getFwdRevCmd());
		CassesroleWebStates.putDouble("Vision Rotate Cmd", visionAlignCTRL.getRotateCmd());
		CassesroleWebStates.putString("Vision_Align_State", visionAlignCTRL.getVisionAlignStateName());
		CassesroleWebStates.putDouble("Vision Actual Yaw at last Frame (deg)", visionAlignCTRL.getGyroAngleAtLastFrame());
		CassesroleWebStates.putDouble("Vision Desired Yaw at last Frame (deg)", visionAlignCTRL.getGyroAngleDesiredAtLastFrame());
		CassesroleWebStates.putString("Vision Cal Last Result", visionDelayCal.getLastResult().toString());
		CassesroleWebStates.putDouble("Vision Cal Avg Delay Time (s)", visionDelayCal.getPrevCalAvgTime());
		CassesroleWebStates.putDouble("Vision Cal Time Std Dev (s)", visionDelayCal.getPrevCalStdDev());
	}
		
	 void updateAllHumanInputs(){
		 
		boolean rising_edge;
		boolean falling_edge;
		
		if( operatorCTRL.Y()){
			shotCTRL.setDesiredShooterState(ShotControl.ShooterStates.PREP_TO_SHOOT);
		}
		
		if(operatorCTRL.A()){
			shotCTRL.setDesiredShooterState(ShotControl.ShooterStates.NO_Shoot);	
		}
		
		if(operatorCTRL.RB()==true & pev_State==false){
			rising_edge=true;	
		} else{
			rising_edge=false;
		}
		
		if(operatorCTRL.RB()==false & pev_State==true){
			falling_edge=true;	
		}
		else{
			falling_edge=false;
		}	
		
		
		if(rising_edge==true){
			shotCTRL.setDesiredShooterState(ShotControl.ShooterStates.SHOOT);	
		} else if(falling_edge==true){
			shotCTRL.setDesiredShooterState(ShotControl.ShooterStates.PREP_TO_SHOOT);
		}
		
		pev_State = operatorCTRL.RB();
		
		
		driverCTRL.updateAirCompEnabled();
		airCompressor.setCompressorEnabled(driverCTRL.getAirCompEnableCmd());
		
		gearSolenoid.set(operatorCTRL.getGearSolenoidCmd());
		
		
		//Set the rumble on if the driver is attempting to align
		// but vision processing isn't aligning
		if(driverCTRL.getAlignDesired() && (visionAlignCTRL.getVisionAlignState() == 0.0)){
			autoAlignNotPossibleDVIndState = (((int)Math.round(Timer.getFPGATimestamp()*1000.0)%RobotConstants.VISION_ALIGN_NOT_ALLOWED_BLINK_PERIOD_MS) > RobotConstants.VISION_ALIGN_NOT_ALLOWED_BLINK_PERIOD_MS/2);
			driverCTRL.setRightRumble(1);
		} else {
			autoAlignNotPossibleDVIndState = false;
			driverCTRL.setRightRumble(0);
		}

		
		//Update Gyro angle
		int angle = gyro.getAngleOffset();
		if(driverCTRL.getGyroReset())
		{
			angle = 0;
		}
		else if(driverCTRL.getGyroReset90())
		{
			angle = 90;
		}
		else if(driverCTRL.getGyroReset180())
		{
			angle = 180;
		}
		else if(driverCTRL.getGyroReset270())
		{
			angle = 270;
		}

		/* Alternate gyro angle update key idea
		 * If decide to try comment out if else above
		 * And update DriverController
		 * 
		 if(driverCTRL.DPadUp())
		 {
			 angle += 90;
			 if (angle >= 360){
				 angle = 0;
			 }
		 } else if(driverCTRL.DPadDown())
		 {
			 angle -= 90;
			 if (angle < 0){
				 angle = 270;
			 }
		 }
		  */
		gyro.setAngleOffset(angle);
		
		/*LED color Selections*/
		if(operatorCTRL.DPadDown()){
			LEDseq.setNoneDesiredPattern();
		} else if(operatorCTRL.DPadUp()){
			LEDseq.setBothDesiredPattern();
		} else if(operatorCTRL.DPadLeft()){
			LEDseq.setGearDesiredPattern();
		} else if(operatorCTRL.DPadRight()){
			LEDseq.setFuelDesiredPattern();
		}
		
	}

	
	
	//Getters & setters for class-scope variables. Needed by MethodHandles logging infrastructure
	public double getLoopTime_ms(){
		return loopTimeElapsed*1000;
	}

	public double getCpuLoad(){
		return ecuStats.totalCPULoadPct;
	}

	public double getRAMUsage(){
		return ecuStats.totalMemUsedPct;
	}
	

	
}

