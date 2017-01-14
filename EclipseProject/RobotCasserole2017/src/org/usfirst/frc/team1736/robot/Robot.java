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
	private double loop_time_elapsed = 0;
	
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
		
		//Set up all logging fields
		CsvLogger.addLoggingFieldDouble("TIME","sec","getFPGATimestamp",Timer.class);
		CsvLogger.addLoggingFieldDouble("batteryvoltage","V","getVoltage", pdp);
		CsvLogger.addLoggingFieldDouble("LoopTime","sec","getLoopTime", this);
		CsvLogger.addLoggingFieldDouble("CpuLoad","%","getCpuLoad", this);
		CsvLogger.addLoggingFieldDouble("RAMUsage","%","getRAMUsage", this);
		
		//Set up and start web server
		webServer = new CasseroleWebServer();
		webServer.startServer();
		botblock.reset();
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
		//Open a new log
		CsvLogger.init();

		botblock.reset();
		loop_time_elapsed = 0;
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		
		//Initialize Timer
		prev_loop_start_timestamp = Timer.getFPGATimestamp();
		
		
		//Log data to file
		CsvLogger.logData(false);

		//Calculate Loop Time
		loop_time_elapsed = Timer.getFPGATimestamp() - prev_loop_start_timestamp;
		
		updateWebStates();
		
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
		

		myRobot.OperatorControl();


		VisionProk.update();
		
		
		
		


		
		CsvLogger.logData(false);
		//Calculate Loop Time
		loop_time_elapsed = Timer.getFPGATimestamp() - prev_loop_start_timestamp;
		
		updateWebStates();
	}

	public double getLoopTime(){
		return loop_time_elapsed;

		

	}

	public double getCpuLoad(){
		return ecuStats.totalCPULoadPct;
	}

	public double getRAMUsage(){
		return ecuStats.totalMemUsedPct;
	}

	 private void updateWebStates(){
		  CassesroleWebStates.putDouble("Loop Time (ms)", loop_time_elapsed*1000);
		  CassesroleWebStates.putDouble("Robot Yaw (deg)", botblock.getYaw());
		  CassesroleWebStates.putDouble("CPU Load (%)", ecuStats.totalCPULoadPct); 
		  CassesroleWebStates.putDouble("RAM Usage (%)", ecuStats.totalMemUsedPct); 
	 }
	
}

