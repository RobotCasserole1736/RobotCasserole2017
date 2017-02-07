package org.usfirst.frc.team1736.robot;


import org.usfirst.frc.team1736.lib.Calibration.CalWrangler;
import org.usfirst.frc.team1736.lib.LoadMon.CasseroleRIOLoadMonitor;
import org.usfirst.frc.team1736.lib.Logging.CsvLogger;
import org.usfirst.frc.team1736.lib.WebServer.CasseroleDriverView;
import org.usfirst.frc.team1736.lib.WebServer.CasseroleWebServer;
import org.usfirst.frc.team1736.lib.WebServer.CassesroleWebStates;
import org.usfirst.frc.team1736.robot.RobotState;

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
	
	// Air pressure
	double currAirPress;
	
	Gyro gyro;
	DriveTrain driveTrain;

	//Vision Processing Algorithm
    VisionProcessing visionProc;
    
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

    //Vision Alignment Control
    VisionAlignment visionAlignCTRL;
    
    //Gear control subsystem (kinda mashed in here just cuz we're lazy)
    Solenoid gearSolenoid;
    
    
    //LED's 
    LEDSequencer LEDseq;
    
    //Compressor & Sensor system
    PneumaticsSupply airCompressor;

    Autonomous auto;
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
		gyro = Gyro.getInstance();
		visionProc = new VisionProcessing(); 
		visionAlignCTRL = VisionAlignment.getInstance();

		ecuStats = new CasseroleRIOLoadMonitor();
		poseCalc = RobotPoseCalculator.getInstance();
		shotCTRL = ShotControl.getInstance();
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
		
		//LEDseq = new LEDSequencer();
		

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
		
		//Update select PID gains from calibrations (only do during disabled to prevent potential gain-switching instability)
		shooterWheelControl.updateGains();
		driveTrain.updateAllCals();
		visionAlignCTRL.updateGains();
		
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
		
		//Run vision alignment algorithm based on vision processing results
		if(visionAlignCTRL.getVisionAlignmentDesired()){
			visionAlignCTRL.GetAligned();
		}
		
		//Update shot control management subsystem
		shotCTRL.update();
		
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
		
		//Update Vision Align Control
		visionAlignCTRL.GetAligned();
		
		auto.update();
		
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
		updateOperatorInputs();
		poseCalc.update();
		
		
		//Update vision processing algorithm to find any targets on in view
		visionProc.update();
		
		//Run vision alignment algorithm based on vision processing results
		if(visionAlignCTRL.getVisionAlignmentDesired()){
			visionAlignCTRL.GetAligned();
		}
		
		//Update shot control management subsystem
		shotCTRL.update();
		
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
		

		//Update user camera
		camGimbal.update();

		//Update Vision Align Control
		visionAlignCTRL.GetAligned();

		
		//Log & display present state data
		updateDriverView();
		CsvLogger.logData(false);
		updateWebStates();
		//Mark end of loop and Calculate Loop Time
		//Must be as close to the end of the loop as possible.
		loopTimeElapsed = Timer.getFPGATimestamp() - prevLoopStartTimestamp;
	}
	
	
	
	
	
	///////////////////////////////////////////////////////////////////
	// Utility Methods
    ///////////////////////////////////////////////////////////////////
	
	//Sets up all the logged channels of data. Should be called once before opening any logs
	public void initLoggingChannels(){
		CsvLogger.addLoggingFieldDouble("TIME","sec","getFPGATimestamp",Timer.class);
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
		CsvLogger.addLoggingFieldDouble("Shooter_Motor_Current","A","getOutputCurrent",shooterWheelControl.shooterTalon);
		CsvLogger.addLoggingFieldDouble("RIO_Loop_Time","msec","getLoopTime_ms", this);
		CsvLogger.addLoggingFieldDouble("RIO_Cpu_Load","%","getCpuLoad", this);
		CsvLogger.addLoggingFieldDouble("RIO_RAM_Usage","%","getRAMUsage", this);
		CsvLogger.addLoggingFieldDouble("Driver_FwdRev_cmd","cmd","getFwdRevCmd", driverCTRL);
		CsvLogger.addLoggingFieldDouble("Driver_Strafe_cmd","cmd","getStrafeCmd", driverCTRL);
		CsvLogger.addLoggingFieldDouble("Driver_Rotate_cmd","cmd","getRotateCmd", driverCTRL);
		CsvLogger.addLoggingFieldBoolean("Driver_Vision_Align_Desired","bit","isVisionAlignmentDesiried",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Auton_DT_FL_Desired_Velocity","RPM","getAutonDtfrontLeftWheelVelocityCmd_rpm", RobotState.class);
		CsvLogger.addLoggingFieldDouble("Auton_DT_FR_Desired_Velocity","RPM","getAutonDtfrontRightWheelVelocityCmd_rpm", RobotState.class);
		CsvLogger.addLoggingFieldDouble("Auton_DT_RL_Desired_Velocity","RPM","getAutonDtrearLeftWheelVelocityCmd_rpm", RobotState.class);
		CsvLogger.addLoggingFieldDouble("Auton_DT_RR_Desired_Velocity","RPM","getAutonDtrearRightWheelVelocityCmd_rpm", RobotState.class);
		CsvLogger.addLoggingFieldBoolean("DT_Running_Closed_Loop","bit","isRunningClosedLoop",driveTrain);
		CsvLogger.addLoggingFieldDouble("Robot_FwdRev_Vel","ft/sec","getRobotFwdRevVel_ftpers",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Robot_Strafe_Vel","ft/sec","getRobotStrafeVel_ftpers",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Robot_FwdRev_Dist","ft","getRobotFwdRevDist_ft",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Robot_Strafe_Dist","ft","getRobotStrafeDist_ft",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Robot_Pose_Angle","deg","getRobotPoseAngle_deg",RobotState.class);
		CsvLogger.addLoggingFieldBoolean("Op_Gear_Release_Desired","bit","isOpGearReleaseDesired",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Shot_State_Command","bits","getopShotCTRLOrdinal",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Hopper_Feed_Cmd","cmd","getHopperMotorCmd", RobotState.class);
		CsvLogger.addLoggingFieldDouble("Intake_Speed_Cmd","cmd","getIntakeSpeedCmd", RobotState.class);
		CsvLogger.addLoggingFieldBoolean("Op_Intake_Desired","bit","isOpIntakeDesired",RobotState.class);
		CsvLogger.addLoggingFieldBoolean("Op_Eject_Desired","bit","isOpEjectDesired",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Climb_Speed_Cmd","cmd","getClimbSpeedCmd", RobotState.class);
		CsvLogger.addLoggingFieldDouble("Shooter_Desired_Velocity","rpm","getShooterDesiredVelocity_rpm",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Shooter_Actual_Velocity","rpm","getShooterActualVelocity_rpm",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Shooter_Motor_Cmd","rpm","getShooterMotorCmd",RobotState.class);
		CsvLogger.addLoggingFieldBoolean("Shooter_Velocity_OK","bit","isShooterVelocityOk",RobotState.class);
		CsvLogger.addLoggingFieldDouble("FL_Motor_Cmd","cmd","getFrontLeftDriveMotorCmd",RobotState.class);
		CsvLogger.addLoggingFieldDouble("FR_Motor_Cmd","cmd","getFrontRightDriveMotorCmd",RobotState.class);
		CsvLogger.addLoggingFieldDouble("RL_Motor_Cmd","cmd","getRearLeftDriveMotorCmd",RobotState.class);
		CsvLogger.addLoggingFieldDouble("RR_Motor_Cmd","cmd","getRearRightDriveMotorCmd",RobotState.class);
		CsvLogger.addLoggingFieldDouble("FL_Wheel_Velocity","rpm","getFrontLeftWheelVelocity_rpm",RobotState.class);
		CsvLogger.addLoggingFieldDouble("FR_Wheel_Velocity","rpm","getFrontRightWheelVelocity_rpm",RobotState.class);
		CsvLogger.addLoggingFieldDouble("RL_Wheel_Velocity","rpm","getRearLeftWheelVelocity_rpm",RobotState.class);
		CsvLogger.addLoggingFieldDouble("RR_Wheel_Velocity","rpm","getRearRightWheelVelocity_rpm",RobotState.class);
		CsvLogger.addLoggingFieldBoolean("Vision_System_Online","bit","isVisionOnline",RobotState.class);
		CsvLogger.addLoggingFieldBoolean("Valid_Vision_Target_Found","bit","isVisionTargetFound",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Vision_Target_Angle_From_Camera","deg","getVisionTargetOffset_deg",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Vision_Target_Gyro_Actual_Angle_At_Frame","deg","getVisionGyroAngleAtLastFrame",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Vision_Target_Gyro_Desired_Angle_At_Frame","deg","getVisionGyroAngleDesiredAtLastFrame",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Vision_Target_Range","ft","getVisionEstTargetDist_ft",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Vision_Process_Time","msec","getProcTimeMs",visionProc.listener);
		CsvLogger.addLoggingFieldDouble("Vision_CoProc_FPS","frames/sec","getVisionCoProcessorFPS",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Vision_CoProc_CPU_load","%","getVisionCoProcessorCPULoad_pct",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Vision_CoProc_Mem_load","%","getVisionCoProcessorMemLoad_pct",RobotState.class);
		CsvLogger.addLoggingFieldBoolean("Vision_Align_Possible","bit","isVisionAlignmentPossible",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Vision_DT_FwdRev_Cmd","cmd","getVisionDtFwdRevCmd", RobotState.class);
		CsvLogger.addLoggingFieldDouble("Vision_DT_Rotate_Cmd","cmd","getVisionDtRotateCmd", RobotState.class);
		CsvLogger.addLoggingFieldBoolean("Vision_Align_On_Target","cmd","isVisionAlignmentOnTarget", RobotState.class);
		CsvLogger.addLoggingFieldDouble("Vision_Align_State", "states", "getVisionAlignState", visionAlignCTRL);
		CsvLogger.addLoggingFieldDouble("Air_Pressure", "psi", "getPress", airCompressor);
		CsvLogger.addLoggingFieldDouble("Compressor_Current", "A", "getCompCurrent", airCompressor);

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
		CasseroleDriverView.newStringBox("Orientation deg");
		CasseroleDriverView.newWebcam("VisionProc_cam", RobotConstants.VISION_PROC_CAMERA_URL, 50, 50, 0);
		CasseroleDriverView.newWebcam("Driver_cam", RobotConstants.DRIVER_CAMERA_URL, 50, 50, 0);

	}
	
	public void updateDriverView(){
		CasseroleDriverView.setDialValue("RobotSpeed ft/sec", poseCalc.getNetSpeedFtPerS());
		CasseroleDriverView.setDialValue("Shooter Speed RPM", shooterWheelControl.getShooterActualVelocityRPM());
		CasseroleDriverView.setDialValue("AirPressure Psi", airCompressor.getPress());
		CasseroleDriverView.setBoolean("Vision Offline", !RobotState.visionOnline);
		CasseroleDriverView.setBoolean("Target in View", RobotState.visionTargetFound && RobotState.visionOnline);
		CasseroleDriverView.setBoolean("Vision Aligning", visionAlignCTRL.getVisionAlignmentDesired() && visionAlignCTRL.getVisionAlignmentPossible() && !visionAlignCTRL.getVisionAlignmentOnTarget());
		CasseroleDriverView.setBoolean("Shooter Spoolup", (shooterWheelControl.getShooterDesiredRPM() > 100) && !(shooterWheelControl.getShooterVelocityOK()));
		
		String temp = String.format("%.1f", gyro.getAngle() % 360.0);
		for(int ii = 0; ii < 5 - temp.length(); ii++){
			temp = " " + temp; 
		}
		CasseroleDriverView.setStringBox("Orientation deg", temp);
		CasseroleDriverView.setWebcamCrosshairs("VisionProc_cam", 
				                                (RobotState.visionTopTgtXPixelPos/RobotConstants.VISION_X_PIXELS) * 100.0, 
				                                (RobotState.visionTopTgtYPixelPos/RobotConstants.VISION_Y_PIXELS) * 100.0);
	}
	
	//Puts all relevant data to the robot State webpage
	public void updateWebStates(){
		CassesroleWebStates.putDouble("Loop Time (ms)",    getLoopTime_ms());
		CassesroleWebStates.putDouble("CPU Load (%)",      getCpuLoad()); 
		CassesroleWebStates.putDouble("RAM Usage (%)",     getRAMUsage()); 
		CassesroleWebStates.putDouble("Driver FwdRev Cmd", driverCTRL.getFwdRevCmd());
		CassesroleWebStates.putDouble("Driver Strafe Cmd", driverCTRL.getStrafeCmd());
		CassesroleWebStates.putDouble("Driver Rotate Cmd", driverCTRL.getRotateCmd());
		CassesroleWebStates.putString("Op Shot Command", shotCTRL.getDesiredShooterState().toString());
		CassesroleWebStates.putDouble("Shooter Wheel Command", shooterWheelControl.getShooterMotorCmd());
		CassesroleWebStates.putDouble("Shooter Desired Speed (RPM)", shooterWheelControl.getShooterDesiredRPM());
		CassesroleWebStates.putDouble("Shooter Actual Speed (RPM)", shooterWheelControl.getShooterActualVelocityRPM());
		CassesroleWebStates.putBoolean("Shooter Speed OK", shooterWheelControl.getShooterVelocityOK());
		CassesroleWebStates.putDouble("Hopper Feed Cmd",   hopControl.getHopperMotorCmd());
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
		CassesroleWebStates.putBoolean("Vision CoProcessor Online", RobotState.visionOnline);
		CassesroleWebStates.putDouble("Vision CoProcessor FPS", RobotState.visionCoProcessorFPS);
		CassesroleWebStates.putDouble("Vision CoProcessor CPU Load (%)", RobotState.visionCoProcessorCPULoad_pct);
		CassesroleWebStates.putDouble("Vision CoProcessor Mem Load (%)", RobotState.visionCoProcessorMemLoad_pct);
		CassesroleWebStates.putDouble("Vision Num Contours Observed", visionProc.listener.getNumTargetsObserved());
		CassesroleWebStates.putBoolean("Vision Target Seen", RobotState.visionTargetFound);
		CassesroleWebStates.putDouble("Vision Target Pixel Pos X", RobotState.visionTopTgtXPixelPos);
		CassesroleWebStates.putDouble("Vision Target Pixel Pos Y", RobotState.visionTopTgtYPixelPos);
		CassesroleWebStates.putDouble("Vision Target Range (ft)", RobotState.visionEstTargetDist_ft);
		CassesroleWebStates.putDouble("Vision Target Offset (deg)", RobotState.visionTargetOffset_deg);
		CassesroleWebStates.putDouble("Vision Heuristic Val", RobotState.visionHeuristicVal);
		CassesroleWebStates.putDouble("Vision Proc Delay (ms)", (Timer.getFPGATimestamp() - RobotState.visionEstCaptureTime)*1000);
		CassesroleWebStates.putDouble("Vision Fwd/Rev Cmd", RobotState.visionDtFwdRevCmd);
		CassesroleWebStates.putDouble("Vision Rotate Cmd", RobotState.visionDtRotateCmd);
		CassesroleWebStates.putDouble("Vision_Align_State", visionAlignCTRL.getVisionAlignState());
		CassesroleWebStates.putDouble("Vision Actual Yaw at last Frame (deg)", RobotState.visionGyroAngleAtLastFrame);
		CassesroleWebStates.putDouble("Vision Desired Yaw at last Frame (deg)", RobotState.visionGyroAngleDesiredAtLastFrame);
	}
		
		//gyro align commands TODO
//		boolean newR = driverCTRL.DPadRight();
//		boolean newD = driverCTRL.DPadDown();
//		boolean newL = driverCTRL.DPadLeft();
//		boolean newU = driverCTRL.DPadUp();
//		RobotState.gyroAlignRight = DaBouncer.AboveDebounceBoo(newR);
//		RobotState.gyroAlignDown = DaBouncer.AboveDebounceBoo(newD);
//		RobotState.gyroAlignUp = driverCTRL.DPadUp();
//		RobotState.gyroAlignLeft = driverCTRL.DPadLeft();
//		
	
	 void updateOperatorInputs(){
		gearSolenoid.set(operatorCTRL.getGearSolenoidCmd());
		airCompressor.setCompressorEnabled(operatorCTRL.getAirCompEnableCmd());
		 
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
		
		if(rising_edge==true){
			shotCTRL.setDesiredShooterState(ShotControl.ShooterStates.SHOOT);	
		}
		
		if(operatorCTRL.RB()==false & pev_State==true){
			falling_edge=true;	
		}
		else{
			falling_edge=false;
		}	
		if(falling_edge==true){
			shotCTRL.setDesiredShooterState(ShotControl.ShooterStates.PREP_TO_SHOOT);
		}
		
		pev_State = operatorCTRL.RB();
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

