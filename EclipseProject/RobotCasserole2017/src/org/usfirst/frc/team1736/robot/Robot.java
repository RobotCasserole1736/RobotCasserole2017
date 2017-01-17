package org.usfirst.frc.team1736.robot;


import org.usfirst.frc.team1736.lib.Calibration.CalWrangler;
import org.usfirst.frc.team1736.lib.LoadMon.CasseroleRIOLoadMonitor;
import org.usfirst.frc.team1736.lib.Logging.CsvLogger;
import org.usfirst.frc.team1736.lib.Sensors.ADIS16448_IMU;
import org.usfirst.frc.team1736.lib.WebServer.CasseroleWebServer;
import org.usfirst.frc.team1736.lib.WebServer.CassesroleWebStates;
import org.usfirst.frc.team1736.vision_processing_2017.Vision_Processing_Main;
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
	
	ADIS16448_IMU botblock;
	DriveTrain myRobot;

	//Vision Processing Algorithm
    Vision_Processing_Main VisionProk;
    
    //Software utilities
    RobotSpeedomitar chris;
    CalWrangler wrangler;
    CasseroleWebServer webServer;

    //Controllers
    Xbox360Controller driverCTRL;
    Xbox360Controller operatorCTRL;

    //Hopper Feed Control
    HopperControl hopControl = new HopperControl(); 

    
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
		botblock =new ADIS16448_IMU();
		VisionProk = new Vision_Processing_Main(); 
		ecuStats = new CasseroleRIOLoadMonitor();
		chris = new RobotSpeedomitar();

		driverCTRL = new Xbox360Controller(0);
		operatorCTRL = new Xbox360Controller(1);
		

		initLoggingChannels();
		
		//Set up and start web server
		webServer = new CasseroleWebServer();
		webServer.startServer();
		
		//Load any saved calibration values
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
		botblock.reset();
		
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
		updateRobotInputs();
		
		//Set Hopper Feed State
		hopControl.setSwitch(true);
		
		//Log & display present state data
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
		botblock.reset();
		
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
		updateRobotInputs();
		
		//Set Hopper Feed State
		hopControl.setSwitch(true);
		
		//Update vision processing algorithm to find any targets on in view
		VisionProk.update();
		

		//Run Drivietrain periodic loop
		myRobot.OperatorControl();
		chris.update();

		
		
		
		


		//Log & display present state data
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
		CsvLogger.addLoggingFieldDouble("batteryvoltage","V","getVoltage", pdp);
		CsvLogger.addLoggingFieldDouble("LoopTime","sec","getLoopTime", this);
		CsvLogger.addLoggingFieldDouble("CpuLoad","%","getCpuLoad", this);
		CsvLogger.addLoggingFieldDouble("RAMUsage","%","getRAMUsage", this);
		CsvLogger.addLoggingFieldDouble("robotFwdRevVel_ftpers","ftperse","getrobotFwdRevVel_ftpers",RobotState.class);
		CsvLogger.addLoggingFieldDouble("robotStrafeVel_ftpers","ftperse","getrobotStrafeVel_ftpers",RobotState.class);
		CsvLogger.addLoggingFieldDouble("HopFeedCmd","%","getHopFeedCmd", this);
	}
	
	//Puts all relevant data to the robot State webpage
	public void updateWebStates(){
		CassesroleWebStates.putDouble("Loop Time (ms)",    getLoopTime()*1000);
		CassesroleWebStates.putDouble("CPU Load (%)",      getCpuLoad()); 
		CassesroleWebStates.putDouble("RAM Usage (%)",     getRAMUsage()); 
		CassesroleWebStates.putDouble("Driver FwdRev Cmd", RobotState.driverFwdRevCmd);
		CassesroleWebStates.putDouble("Driver Strafe Cmd", RobotState.driverStrafeCmd);
		CassesroleWebStates.putDouble("Driver Rotate Cmd", RobotState.driverRotateCmd);
		CassesroleWebStates.putDouble("Hopper Feed Cmd",   RobotState.hopperMotorCmd);
		CassesroleWebStates.putDouble("Robot Yaw (deg)",   RobotState.robotPoseAngle_deg);
		CassesroleWebStates.putDouble("Front Left Motor Output",   RobotState.frontLeftDriveMotorCmd);
		CassesroleWebStates.putDouble("Front Right Motor Output",   RobotState.frontRightDriveMotorCmd);
		CassesroleWebStates.putDouble("Rear Left Motor Output",   RobotState.rearLeftDriveMotorCmd);
		CassesroleWebStates.putDouble("Rear Right Motor Output",   RobotState.rearRightDriveMotorCmd);

	}

	//Updates all relevant robot inputs. Should be called during periodic loops
	public void updateRobotInputs(){
		RobotState.driverFwdRevCmd = driverCTRL.LStick_Y();
		RobotState.driverStrafeCmd = driverCTRL.LStick_X();
		RobotState.driverRotateCmd = driverCTRL.RStick_X();
		RobotState.frontLeftDriveMotorCmd = myRobot.getFLDriveMotorCmd();
		RobotState.frontRightDriveMotorCmd = myRobot.getFRDriveMotorCmd();
		RobotState.rearLeftDriveMotorCmd = myRobot.getRLDriveMotorCmd();
		RobotState.rearRightDriveMotorCmd = myRobot.getRRDriveMotorCmd();
		RobotState.robotPoseAngle_deg = botblock.getYaw();
	}

	
	
	//Getters & setters for class-scope variables. Needed by MethodHandles logging infrastructure
	public double getLoopTime(){
		return loop_time_elapsed;
	}

	public double getCpuLoad(){
		return ecuStats.totalCPULoadPct;
	}

	public double getRAMUsage(){
		return ecuStats.totalMemUsedPct;
	}

	public double getHopFeedCmd(){
		return  RobotState.hopperMotorCmd;
	}

	
}

