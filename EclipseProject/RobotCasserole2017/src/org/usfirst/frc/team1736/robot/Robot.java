package org.usfirst.frc.team1736.robot;


import org.usfirst.frc.team1736.lib.Calibration.CalWrangler;
import org.usfirst.frc.team1736.lib.LoadMon.CasseroleRIOLoadMonitor;
import org.usfirst.frc.team1736.lib.Logging.CsvLogger;
import org.usfirst.frc.team1736.lib.Sensors.ADXRS453_Gyro;
import org.usfirst.frc.team1736.lib.WebServer.CasseroleDriverView;
import org.usfirst.frc.team1736.lib.WebServer.CasseroleWebServer;
import org.usfirst.frc.team1736.lib.WebServer.CassesroleWebStates;
import org.usfirst.frc.team1736.lib.HAL.JoyStickScaler;
import org.usfirst.frc.team1736.lib.HAL.Xbox360Controller;
import org.usfirst.frc.team1736.robot.RobotState;



import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
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
	private double prev_loop_start_timestamp;
	double loop_time_elapsed;
	CasseroleRIOLoadMonitor ecuStats;
	
	// Physical Devices on the robot
	PowerDistributionPanel pdp;
	
	ADXRS453_Gyro gyro;
	DriveTrain myRobot;

	//Vision Processing Algorithm
    Vision_Processing_Main VisionProk;
    
    //Use Vision output to get drive settings to reach alignment
    VisionAlignment VisionAlign;
    
    //Software utilities
    RobotSpeedomitar chris;
    CalWrangler wrangler;
    CasseroleWebServer webServer;

    //Controllers
    Xbox360Controller driverCTRL;
    Xbox360Controller operatorCTRL;

    //Hopper Feed Control
    HopperControl hopControl; 
    
    //Intake Control
    IntakeControl intakeControl;
    
    //Shooter wheel control
    ShooterWheelCTRL shooterControl;
    
    //Climber Control
    ClimberControl climbControl;
    
    boolean pev_State;
    Sht_ctrl shotCTRL;
    
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
		myRobot = new DriveTrain();
		pdp = new PowerDistributionPanel();
		gyro =new ADXRS453_Gyro();
		VisionProk = new Vision_Processing_Main(); 
		VisionAlign = new VisionAlignment();
		ecuStats = new CasseroleRIOLoadMonitor();
		chris = new RobotSpeedomitar();
		shotCTRL = new Sht_ctrl();
		hopControl = new HopperControl();
		shooterControl = new ShooterWheelCTRL();
		climbControl = new ClimberControl();
		intakeControl = new IntakeControl();

		driverCTRL = new Xbox360Controller(0);
		operatorCTRL = new Xbox360Controller(1);
		driverCTRL.setDeadzone(0.175);
		operatorCTRL.setDeadzone(0.175);
		

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
	
	@Override
	public void disabledPeriodic() {
		
		//Mark start of loop, Initialize Timer
		//Must be as close to the start of the loop as possible
		prev_loop_start_timestamp = Timer.getFPGATimestamp();
		
		//Get all inputs from outside the robot
		updateGlobalSensorInputs();
		chris.update();
		
		//Update vision processing algorithm to find any targets on in view
		VisionProk.update();
		if(RobotState.visionAlignmentDesiried){
			VisionAlign.GetAligned();
		}
		
		//Update select PID gains from calibrations (only do during disabled to prevent potential gain-switching instability)
		shooterControl.updateGains();
		myRobot.updateAllCals();
		
		updateDriverView();
		updateWebStates();

		//Mark end of loop and Calculate Loop Time
		//Must be as close to the end of the loop as possible.
		loop_time_elapsed = Timer.getFPGATimestamp() - prev_loop_start_timestamp;
		
	}
		
		
		
		
		
		
		
		
	///////////////////////////////////////////////////////////////////
	// Autonomous top-level Methods
    ///////////////////////////////////////////////////////////////////
	/**
	 * This method gets run right before the robot starts running autonomous mode.
	 */
	@Override
	public void autonomousInit() {	
		myRobot.myDrive.setSafetyEnabled(false);
		
		loop_time_elapsed = 0;

		//Assume starting at 0 degrees
		gyro.reset();
		
		myRobot.resetAllEncoders();
		myRobot.resetAllIntegrators();
		
		//Open a new log
		CsvLogger.init();

	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		
		//Mark start of loop, Initialize Timer
		//Must be as close to the start of the loop as possible
		prev_loop_start_timestamp = Timer.getFPGATimestamp();
		
		//Get all inputs from outside the robot
		updateGlobalSensorInputs();
		chris.update();
		myRobot.readEncoders();
		
		//Update vision processing algorithm to find any targets on in view
		VisionProk.update();
		
		if(RobotState.visionAlignmentDesiried){
			VisionAlign.GetAligned();
		}
		shotCTRL.update();
		//Update Hopper Control
		hopControl.update();
		
		//Update Intake Control
		intakeControl.update();
		
		//Update shooter wheel control
		shooterControl.update();
		
		myRobot.autonomousControl();
		
		//Update Climber Control
		climbControl.update();
		
		//Log & display present state data
		updateDriverView();
		//CsvLogger.logData(false);
		updateWebStates();

		//Mark end of loop and Calculate Loop Time
		//Must be as close to the end of the loop as possible.
		loop_time_elapsed = Timer.getFPGATimestamp() - prev_loop_start_timestamp;
	}
	
	
	
	
	
	
	
	///////////////////////////////////////////////////////////////////
	// Teleoperated top-level Methods
    ///////////////////////////////////////////////////////////////////
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopInit() {
		myRobot.myDrive.setSafetyEnabled(false);

		loop_time_elapsed = 0;
		
		myRobot.resetAllIntegrators();
		
		//Open a new log
		CsvLogger.init();
	}	
	
	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		//Initialize Timer
		prev_loop_start_timestamp = Timer.getFPGATimestamp();
		
		//Get all inputs from outside the robot
		updateDriverInputs();
		updateOperatorInputs();
		updateGlobalSensorInputs();
		chris.update();
		myRobot.readEncoders();
		
		//Update vision processing algorithm to find any targets on in view
		VisionProk.update();
		if(RobotState.visionAlignmentDesiried){
			VisionAlign.GetAligned();
			
		}
		shotCTRL.update();
		//Update Hopper Control
		hopControl.update();
		
		//Update Intake Control
		intakeControl.update();
		
		//Update shooter wheel control
		shooterControl.update();
		
		//Update Climber Control
		climbControl.update();
		
		//Run Drivietrain periodic loop
		myRobot.operatorControl();
		
		//Log & display present state data
		updateDriverView();
		CsvLogger.logData(false);
		updateWebStates();
		//Mark end of loop and Calculate Loop Time
		//Must be as close to the end of the loop as possible.
		loop_time_elapsed = Timer.getFPGATimestamp() - prev_loop_start_timestamp;
	}
	
	
	
	
	
	///////////////////////////////////////////////////////////////////
	// Utility Methods
    ///////////////////////////////////////////////////////////////////
	
	//Sets up all the logged channels of data. Should be called once before opening any logs
	public void initLoggingChannels(){
		CsvLogger.addLoggingFieldDouble("TIME","sec","getFPGATimestamp",Timer.class);
		CsvLogger.addLoggingFieldDouble("PDP_Voltage","V","getVoltage", pdp);
		CsvLogger.addLoggingFieldDouble("PDP_Total_Current","A","getTotalCurrent", pdp);
		CsvLogger.addLoggingFieldDouble("PDP_DT_FL_Current","A","getCurrent", pdp, RobotIOMap.DRIVETRAIN_FRONT_LEFT_PDP_CH);
		CsvLogger.addLoggingFieldDouble("PDP_DT_FR_Current","A","getCurrent", pdp, RobotIOMap.DRIVETRAIN_FRONT_RIGHT_PDP_CH);
		CsvLogger.addLoggingFieldDouble("PDP_DT_RL_Current","A","getCurrent", pdp, RobotIOMap.DRIVETRAIN_REAR_LEFT_PDP_CH);
		CsvLogger.addLoggingFieldDouble("PDP_DT_RR_Current","A","getCurrent", pdp, RobotIOMap.DRIVETRAIN_REAR_RIGHT_PDP_CH);
		CsvLogger.addLoggingFieldDouble("Hopper_Motor_Current","A","getCurrent", pdp,  RobotIOMap.HOPPER_MOTOR_PDP_CH);
		CsvLogger.addLoggingFieldDouble("Climber_Motor_Current","A","getCurrent", pdp, RobotIOMap.CLIMBER_MOTOR_PDP_CH);
		CsvLogger.addLoggingFieldDouble("Intake_Motor_Current","A","getCurrent", pdp,  RobotIOMap.INTAKE_MOTOR_PDP_CH);
		CsvLogger.addLoggingFieldDouble("Shooter_Motor_Current","A","getOutputCurrent",shooterControl.TallonFlame);
		CsvLogger.addLoggingFieldDouble("RIO_Loop_Time","msec","getLoopTime_ms", this);
		CsvLogger.addLoggingFieldDouble("RIO_Cpu_Load","%","getCpuLoad", this);
		CsvLogger.addLoggingFieldDouble("RIO_RAM_Usage","%","getRAMUsage", this);
		CsvLogger.addLoggingFieldDouble("Driver_FwdRev_cmd","cmd","getDriverFwdRevCmd", RobotState.class);
		CsvLogger.addLoggingFieldDouble("Driver_Strafe_cmd","cmd","getDriverStrafeCmd", RobotState.class);
		CsvLogger.addLoggingFieldDouble("Driver_Rotate_cmd","cmd","getDriverRotateCmd", RobotState.class);
		CsvLogger.addLoggingFieldBoolean("Driver_Vision_Align_Desired","bit","isVisionAlignmentDesiried",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Auton_DT_FL_Desired_Velocity","RPM","getAutonDtfrontLeftWheelVelocityCmd_rpm", RobotState.class);
		CsvLogger.addLoggingFieldDouble("Auton_DT_FR_Desired_Velocity","RPM","getAutonDtfrontRightWheelVelocityCmd_rpm", RobotState.class);
		CsvLogger.addLoggingFieldDouble("Auton_DT_RL_Desired_Velocity","RPM","getAutonDtrearLeftWheelVelocityCmd_rpm", RobotState.class);
		CsvLogger.addLoggingFieldDouble("Auton_DT_RR_Desired_Velocity","RPM","getAutonDtrearRightWheelVelocityCmd_rpm", RobotState.class);
		CsvLogger.addLoggingFieldBoolean("DT_Running_Closed_Loop","bit","isRunningClosedLoop",myRobot);
		CsvLogger.addLoggingFieldDouble("Robot_FwdRev_Vel","ft/sec","getRobotFwdRevVel_ftpers",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Robot_Strafe_Vel","ft/sec","getRobotStrafeVel_ftpers",RobotState.class);
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
		CsvLogger.addLoggingFieldDouble("Vision_Target_Angle","deg","getVisionTargetOffset_deg",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Vision_Target_Range","ft","getVisionEstTargetDist_ft",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Vision_Process_Time","msec","getProcTimeMs",VisionProk.VL);
		CsvLogger.addLoggingFieldDouble("Vision_CoProc_FPS","frames/sec","getVisionCoProcessorFPS",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Vision_CoProc_CPU_load","%","getVisionCoProcessorCPULoad_pct",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Vision_CoProc_Mem_load","%","getVisionCoProcessorMemLoad_pct",RobotState.class);
		CsvLogger.addLoggingFieldBoolean("Vision_Align_Possible","bit","isVisionAlignmentPossible",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Vision_DT_FwdRev_Cmd","cmd","getVisionDtFwdRevCmd", RobotState.class);
		CsvLogger.addLoggingFieldDouble("Vision_DT_Rotate_Cmd","cmd","getVisionDtRotateCmd", RobotState.class);
		CsvLogger.addLoggingFieldBoolean("Vision_Align_On_Target","cmd","isVisionAlignmentOnTarget", RobotState.class);
	
	}
	
	public void initDriverView(){
		CasseroleDriverView.newDial("RobotSpeed ft/sec", 0, 25, 5, 0, 20);
		CasseroleDriverView.newBoolean("Vision Offline", "red");
		CasseroleDriverView.newBoolean("Target in View", "green");
		CasseroleDriverView.newStringBox("Orientation deg");
		CasseroleDriverView.newWebcam("VisionProc_cam", "http://10.17.36.11/mjpg/video.mjpg", 50, 50, 0);
		

	}
	
	//Puts all relevant data to the robot State webpage
	public void updateWebStates(){
		CassesroleWebStates.putDouble("Loop Time (ms)",    getLoopTime_ms());
		CassesroleWebStates.putDouble("CPU Load (%)",      getCpuLoad()); 
		CassesroleWebStates.putDouble("RAM Usage (%)",     getRAMUsage()); 
		CassesroleWebStates.putDouble("Driver FwdRev Cmd", RobotState.driverFwdRevCmd);
		CassesroleWebStates.putDouble("Driver Strafe Cmd", RobotState.driverStrafeCmd);
		CassesroleWebStates.putDouble("Driver Rotate Cmd", RobotState.driverRotateCmd);
		CassesroleWebStates.putString("Op Shot Command", RobotState.opShotCTRL.toString());
		CassesroleWebStates.putDouble("Shooter Wheel Command", RobotState.shooterMotorCmd);
		CassesroleWebStates.putDouble("Shooter Desired Speed (RPM)", RobotState.shooterDesiredVelocity_rpm);
		CassesroleWebStates.putDouble("Shooter Actual Speed (RPM)", RobotState.shooterActualVelocity_rpm);
		CassesroleWebStates.putBoolean("Shooter Speed OK", RobotState.shooterVelocityOk);
		CassesroleWebStates.putDouble("Hopper Feed Cmd",   RobotState.hopperMotorCmd);
		CassesroleWebStates.putDouble("Intake Speed Cmd",   RobotState.intakeSpeedCmd);
		CassesroleWebStates.putDouble("Climb Speed Cmd",   RobotState.climbSpeedCmd);
		CassesroleWebStates.putDouble("Robot Yaw (deg)",   RobotState.robotPoseAngle_deg);
		CassesroleWebStates.putDouble("Front Left Motor Output",   RobotState.frontLeftDriveMotorCmd);
		CassesroleWebStates.putDouble("Front Right Motor Output",   RobotState.frontRightDriveMotorCmd);
		CassesroleWebStates.putDouble("Rear Left Motor Output",   RobotState.rearLeftDriveMotorCmd);
		CassesroleWebStates.putDouble("Rear Right Motor Output",   RobotState.rearRightDriveMotorCmd);
		CassesroleWebStates.putDouble("Front Left Motor Speed (RPM)",   RobotState.frontLeftWheelVelocity_rpm);
		CassesroleWebStates.putDouble("Front Right Motor Speed (RPM)",   RobotState.frontRightWheelVelocity_rpm);
		CassesroleWebStates.putDouble("Rear Left Motor Speed (RPM)",   RobotState.rearLeftWheelVelocity_rpm);
		CassesroleWebStates.putDouble("Rear Right Motor Speed (RPM)",   RobotState.rearRightWheelVelocity_rpm);
		CassesroleWebStates.putBoolean("Vision CoProcessor Online", RobotState.visionOnline);
		CassesroleWebStates.putDouble("Vision CoProcessor FPS", RobotState.visionCoProcessorFPS);
		CassesroleWebStates.putDouble("Vision CoProcessor CPU Load (%)", RobotState.visionCoProcessorCPULoad_pct);
		CassesroleWebStates.putDouble("Vision CoProcessor Mem Load (%)", RobotState.visionCoProcessorMemLoad_pct);
		CassesroleWebStates.putDouble("Vision Num Contours Observed", VisionProk.VL.getNumTargetsObserved());
		CassesroleWebStates.putBoolean("Vision Target Seen", RobotState.visionTargetFound);
		CassesroleWebStates.putDouble("Vision Target Pixel Pos X", RobotState.visionTopTgtXPixelPos);
		CassesroleWebStates.putDouble("Vision Target Pixel Pos Y", RobotState.visionTopTgtYPixelPos);
		CassesroleWebStates.putDouble("Vision Heuristic Val", RobotState.visionHeuristicVal);
		CassesroleWebStates.putDouble("Vision Proc Delay (ms)", (Timer.getFPGATimestamp() - RobotState.visionEstCaptureTime)*1000);
		
		
	}
	
	public void updateDriverView(){
		CasseroleDriverView.setDialValue("RobotSpeed ft/sec", RobotState.robotNetSpeed_ftpers);
		CasseroleDriverView.setBoolean("Vision Offline", !RobotState.visionOnline);
		CasseroleDriverView.setBoolean("Target in View", RobotState.visionTargetFound);
		
		String temp = String.format("%.1f", RobotState.robotPoseAngle_deg % 360.0);
		for(int ii = 0; ii < 5 - temp.length(); ii++){
			temp = " " + temp; 
		}
		CasseroleDriverView.setStringBox("Orientation deg", temp);
	}

	//Updates all relevant robot inputs. Should be called during periodic loops
	public void updateDriverInputs(){
		//drive train commands
		RobotState.driverFwdRevCmd = JoyStickScaler.joyscale(driverCTRL.LStick_Y());
		RobotState.driverStrafeCmd = JoyStickScaler.joyscale(driverCTRL.LStick_X());
		RobotState.driverRotateCmd = JoyStickScaler.joyscale(driverCTRL.RStick_X());
		RobotState.visionAlignmentDesiried = driverCTRL.RB();	
		
		//camera positioning commands
		RobotState.gearCamAlign = driverCTRL.B();
		RobotState.intakeCamAlign = driverCTRL.X();
		RobotState.shooterCamAlign = driverCTRL.Y();
		
		//gyro align commands
//		boolean newR = driverCTRL.DPadRight();
//		boolean newD = driverCTRL.DPadDown();
//		boolean newL = driverCTRL.DPadLeft();
//		boolean newU = driverCTRL.DPadUp();
//		RobotState.gyroAlignRight = DaBouncer.AboveDebounceBoo(newR);
//		RobotState.gyroAlignDown = DaBouncer.AboveDebounceBoo(newD);
//		RobotState.gyroAlignUp = driverCTRL.DPadUp();
//		RobotState.gyroAlignLeft = driverCTRL.DPadLeft();
//		
		
	}
	
	 void updateOperatorInputs(){
		boolean rising_edge;
		boolean falling_edge;
		
		RobotState.climbSpeedCmd = operatorCTRL.LStick_X();
		RobotState.climbEnable = true;
		
		RobotState.opIntakeDesired = operatorCTRL.LB();
		RobotState.opEjectDesired = operatorCTRL.B();
		
		if( operatorCTRL.Y()){
			RobotState.opShotCTRL=Shooter_States.PREP_TO_SHOOT;
		}
		
		if(operatorCTRL.A()){
			RobotState.opShotCTRL=Shooter_States.NO_Shoot;	
		}
		
		if(operatorCTRL.RB()==true & pev_State==false){
			rising_edge=true;	
		} else{
			rising_edge=false;
		}
		
		if(rising_edge==true){
			RobotState.opShotCTRL=Shooter_States.SHOOT;	
		}
		
		if(operatorCTRL.RB()==false & pev_State==true){
			falling_edge=true;	
		}
		else{
			falling_edge=false;
		}	
		if(falling_edge==true){
			RobotState.opShotCTRL=Shooter_States.PREP_TO_SHOOT;
		}
		
		pev_State = operatorCTRL.RB();
	}
	
	public void updateGlobalSensorInputs(){
		RobotState.robotPoseAngle_deg = gyro.getAngle();
	}

	
	
	//Getters & setters for class-scope variables. Needed by MethodHandles logging infrastructure
	public double getLoopTime_ms(){
		return loop_time_elapsed*1000;
	}

	public double getCpuLoad(){
		return ecuStats.totalCPULoadPct;
	}

	public double getRAMUsage(){
		return ecuStats.totalMemUsedPct;
	}
	

	
}

