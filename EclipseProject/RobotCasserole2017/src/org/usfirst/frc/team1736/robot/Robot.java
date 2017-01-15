package org.usfirst.frc.team1736.robot;


import org.usfirst.frc.team1736.lib.LoadMon.CasseroleRIOLoadMonitor;
import org.usfirst.frc.team1736.lib.Logging.CsvLogger;
import org.usfirst.frc.team1736.lib.Sensors.ADIS16448_IMU;
import org.usfirst.frc.team1736.lib.WebServer.CasseroleWebServer;
import org.usfirst.frc.team1736.lib.WebServer.CassesroleWebStates;
import org.usfirst.frc.team1736.vision_processing_2017.Vision_Processing_Main;


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
	
	//Performance Timer
	Timer autoTimer = new Timer();
	private double prev_loop_start_timestamp = 0;
	double loop_time_elapsed = 0;
	
	//Processor Stats
	CasseroleRIOLoadMonitor ecuStats = new CasseroleRIOLoadMonitor();
	
	// Physical Devices on the robot
	PowerDistributionPanel pdp;
	public CasseroleWebServer webServer;
	ADIS16448_IMU botblock;
	DriveTrain myRobot;

	
    Vision_Processing_Main VisionProk;
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
		
		initLoggingChannels();
		
		//Set up and start web server
		webServer = new CasseroleWebServer();
		webServer.startServer();
		
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
		

		//Update vision processing algorithm to find any targets on in view
		VisionProk.update();
		
		//Run Drivietrain periodic loop
		myRobot.OperatorControl();


		
		
		
		


		//Log & display present state data
		CsvLogger.logData(false);
		updateWebStates();
		//Mark end of loop and Calculate Loop Time
		//Must be as close to the end of the loop as possible.
		loop_time_elapsed = Timer.getFPGATimestamp() - prev_loop_start_timestamp;
	}
	
	
	
	//Sets up all the logged channels of data. Should be called once before opening any logs
	public void initLoggingChannels(){
		CsvLogger.addLoggingFieldDouble("TIME","sec","getFPGATimestamp",Timer.class);
		CsvLogger.addLoggingFieldDouble("batteryvoltage","V","getVoltage", pdp);
		CsvLogger.addLoggingFieldDouble("LoopTime","sec","getLoopTime", this);
		CsvLogger.addLoggingFieldDouble("CpuLoad","%","getCpuLoad", this);
		CsvLogger.addLoggingFieldDouble("RAMUsage","%","getRAMUsage", this);
		
	}
	
	//Puts all relevant data to the 
	public void updateWebStates(){
		CassesroleWebStates.putDouble("Loop Time (ms)",    loop_time_elapsed*1000);
		CassesroleWebStates.putDouble("Robot Yaw (deg)",   botblock.getYaw());
		CassesroleWebStates.putDouble("CPU Load (%)",      ecuStats.totalCPULoadPct); 
		CassesroleWebStates.putDouble("RAM Usage (%)",     ecuStats.totalMemUsedPct); 
		CassesroleWebStates.putDouble("Driver FwdRev Cmd", RobotState.driverFwdRevCmd);
		CassesroleWebStates.putDouble("Driver Strafe Cmd", RobotState.driverStrafeCmd);
		CassesroleWebStates.putDouble("Driver Rotate Cmd", RobotState.driverRotateCmd);
	}

	//Updates all relevant robot inputs. Should be called during periodic loops
	public void updateRobotInputs(){
		
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

	
}

