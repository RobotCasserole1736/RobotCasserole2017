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
import org.usfirst.frc.team1736.lib.WebServer.CasseroleWebServer;
import org.usfirst.frc.team1736.lib.WebServer.CassesroleWebStates;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	CalWrangler wrangler;
	CasseroleWebServer webServer;
	Compressor compressor;
	AnalogInput analogInput;
	RobotDrive drive;
	XboxController xbox; 
	Spark sparky;
	XboxController shooterController;
	CANTalon shooterMotor;
	Servo flappyGear; 


	///////////////////////////////////////////////////////////////////
	// Robot Top-Level Methods
	///////////////////////////////////////////////////////////////////
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Set up and start web server (must be after all other website init functions)
		webServer = new CasseroleWebServer();
		webServer.startServer();
		flappyGear = new Servo(8);
		flappyGear.setAngle(-90);
		
		CassesroleWebStates.putDouble("Time since boot (s)", 0.0);

		drive = new RobotDrive(1,3,0,2);

		xbox = new XboxController (0);

		analogInput = new AnalogInput(0);
		
		compressor = new Compressor();


		sparky = new Spark(5);
		
		shooterController = new XboxController(0);
		shooterMotor = new CANTalon(0);
		double Shooter_ff_Gain = 0.0269;
		double Shooter_P_Gain = 0.6;
		double Shooter_I_Gain = 0.01;
		double Shooter_D_Gain = 0.2;
		double ErrorRange = 100;

		shooterMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative); // Tells the SRX an encoder is attached to its input.
		shooterMotor.setProfile(0); // Select slot 0 for PID gains
		shooterMotor.changeControlMode(TalonControlMode.Speed); // Set that a PID algorithm should be used to control the output

		shooterMotor.setF(Shooter_ff_Gain); // Set the PID algorithm gains based on calibration values
		shooterMotor.setP(Shooter_P_Gain);
		shooterMotor.setI(Shooter_I_Gain);
		shooterMotor.setD(Shooter_D_Gain);

		shooterMotor.enableBrakeMode(true); // Brake when 0 commanded (to stop shooter as fast as possible)
		shooterMotor.reverseOutput(false);
		shooterMotor.reverseSensor(true);
		shooterMotor.setIZone(100);
		
	}

	/**
	 * This function is called just before the robot enters disabled
	 */
	@Override
	public void disabledInit() {

	}

	/**
	 * This function is called periodically while disabled. Note no motors should be commanded on while in this state.
	 */
	@Override
	public void disabledPeriodic() {

	}

	// /////////////////////////////////////////////////////////////////
	// Autonomous top-level Methods
	// /////////////////////////////////////////////////////////////////
	/**
	 * This method gets run right before the robot starts running autonomous mode.
	 */
	@Override
	public void autonomousInit() {

	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {

	}

	// /////////////////////////////////////////////////////////////////
	// Teleoperated top-level Methods
	// /////////////////////////////////////////////////////////////////
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	
	
	public void teleopInit() {

		
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {

		CassesroleWebStates.putDouble("Time since boot (s)", Timer.getFPGATimestamp());
		drive.arcadeDrive(xbox,
                1,
                xbox,
                4,
                true);

		boolean flappyGearDesired = xbox.getTrigger(GenericHID.Hand.kRight);
		
		if(flappyGearDesired = true){
			flappyGear.setAngle(90);
		}else{
			flappyGear.setAngle(-90);
		}
	


		double voltage = analogInput.getVoltage();
		double pressure = (voltage * 37.5) - 18.75;
		
		CassesroleWebStates.putDouble("Pressure", pressure);
		
		

		boolean hammerOnX = xbox.getXButton();
		CassesroleWebStates.putBoolean("x button value",hammerOnX );
		if (hammerOnX == true){
			sparky.set(1);
		}
		else {
			sparky.set(0);
		}


		boolean aButton = shooterController.getAButton();
		if(aButton) {
			 shooterMotor.setSetpoint(1000);
		}
		else {
			shooterMotor.setSetpoint(0);
		}
		CassesroleWebStates.putDouble("Shooter Speed (rpm)", shooterMotor.getSpeed());

	}
}
