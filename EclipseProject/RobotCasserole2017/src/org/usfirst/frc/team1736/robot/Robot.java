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
    
    //Shooter wheel control
    ShooterWheelCTRL shooterControl;
    
    //Climber Control
    ClimberControl climbControl;

    
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
		hopControl = new HopperControl();
		shooterControl = new ShooterWheelCTRL();
		climbControl = new ClimberControl();

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
		
		//Update shooter PID gains from calibrations (only do during disabled to prevent potential gain-switching instability)
		shooterControl.updateGains();
		
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
				
		loop_time_elapsed = 0;

		//Assume starting at 0 degrees
		gyro.reset();
		
		myRobot.resetEncoders();
		
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
		
		//Update Hopper Control
		hopControl.update();
		
		//Update shooter wheel control
		shooterControl.update();
		
		myRobot.autonomousControl();
		
		//Update Climber Control
		climbControl.update();
		
		//Log & display present state data
		updateDriverView();
		CsvLogger.logData(false);
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
		

		loop_time_elapsed = 0;
		
		//Assume starting at 0 degrees
		gyro.reset();
		
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
		
		//Update Hopper Control
		hopControl.update();
		
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
		CsvLogger.addLoggingFieldDouble("Auton_FwdRev_cmd","cmd","getAutonDtFwdRevCmd", RobotState.class);
		CsvLogger.addLoggingFieldDouble("Auton_Strafe_cmd","cmd","getAutonDtrStrafeCmd", RobotState.class);
		CsvLogger.addLoggingFieldDouble("Auton_Rotate_cmd","cmd","getAutonDtRotateCmd", RobotState.class);
		CsvLogger.addLoggingFieldDouble("Robot_FwdRev_Vel","ft/sec","getRobotFwdRevVel_ftpers",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Robot_Strafe_Vel","ft/sec","getRobotStrafeVel_ftpers",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Robot_Pose_Angle","deg","getRobotPoseAngle_deg",RobotState.class);
		CsvLogger.addLoggingFieldBoolean("Op_Gear_Release_Desired","bit","isOpGearReleaseDesired",RobotState.class);
		CsvLogger.addLoggingFieldBoolean("Op_Prep_to_Shoot_Desired","bit","isOpPrepToShootDesired",RobotState.class);
		CsvLogger.addLoggingFieldBoolean("Op_Shot_Desired","bit","isOpShotDesired",RobotState.class);
		CsvLogger.addLoggingFieldDouble("Hopper_Feed_Cmd","cmd","getHopperMotorCmd", RobotState.class);
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
		CsvLogger.addLoggingFieldBoolean("Vision_Align_Active","bit","isVisionAlignmentActive",RobotState.class);
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

	}
	
	//Puts all relevant data to the robot State webpage
	public void updateWebStates(){
		CassesroleWebStates.putDouble("Loop Time (ms)",    getLoopTime_ms());
		CassesroleWebStates.putDouble("CPU Load (%)",      getCpuLoad()); 
		CassesroleWebStates.putDouble("RAM Usage (%)",     getRAMUsage()); 
		CassesroleWebStates.putDouble("Driver FwdRev Cmd", RobotState.driverFwdRevCmd);
		CassesroleWebStates.putDouble("Driver Strafe Cmd", RobotState.driverStrafeCmd);
		CassesroleWebStates.putDouble("Driver Rotate Cmd", RobotState.driverRotateCmd);
		CassesroleWebStates.putDouble("Hopper Feed Cmd",   RobotState.hopperMotorCmd);
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
		
		RobotState.driverFwdRevCmd = JoyStickScaler.joyscale(driverCTRL.LStick_Y());

		RobotState.driverStrafeCmd = JoyStickScaler.joyscale(driverCTRL.LStick_X());
		RobotState.driverRotateCmd = JoyStickScaler.joyscale(driverCTRL.RStick_X());
	}
	
	public void updateOperatorInputs(){
		RobotState.climbSpeedCmd = operatorCTRL.LStick_Y();
		RobotState.climbEnable = true;
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

