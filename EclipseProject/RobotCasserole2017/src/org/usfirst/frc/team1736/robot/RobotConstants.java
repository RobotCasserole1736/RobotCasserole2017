package org.usfirst.frc.team1736.robot;

import java.awt.Color;

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

public class RobotConstants {
	
	///////////////////////////////////////////////////////////////////////////
	// Networking Architecture Constants
	///////////////////////////////////////////////////////////////////////////
	//Connection parameters for listening for coprocessor results
	public static final String COPPROCESSOR_LISTEN_ADDRESS = "10.17.36.9";
	public static final int	COPROCESSOR_LISTEN_PORT = 5800;
	public static final String DRIVER_CAMERA_URL = "http://10.17.36.8/mjpg/video.mjpg";
	public static final double EXPECTED_NETWORK_LATENCY_SEC = ((0.345+0.275)*0.5)/1000.0; //An educated guess, based off of measurements with "ping"
	
	
	
	///////////////////////////////////////////////////////////////////////////
	// Electrical IO Map
	///////////////////////////////////////////////////////////////////////////
	
	//Motor Output port mappings
	public static final int DRIVETRAIN_FRONT_RIGHT_MOTOR = 0;
	public static final int DRIVETRAIN_FRONT_LEFT_MOTOR  = 1;
	public static final int DRIVETRAIN_REAR_RIGHT_MOTOR  = 2;
	public static final int DRIVETRAIN_REAR_LEFT_MOTOR   = 3;
	public static final int HOPPER_MOTOR_PWM_PORT  = 4;
	public static final int CLIMBER_MOTOR1_PWM_PORT = 5;
	public static final int CLIMBER_MOTOR2_PWM_PORT = 6;
	public static final int INTAKE_MOTOR_PWM_PORT  = 7;

	public static final int CAMERA_TILT_SERVO_PWM_PORT = 8; 
	public static final int CAMERA_PAN_SERVO_PWM_PORT = 9; 
	
	//Motor PDP channel mappings
	public static final int DRIVETRAIN_FRONT_RIGHT_PDP_CH = 0;
	public static final int DRIVETRAIN_FRONT_LEFT_PDP_CH  = 1;
	public static final int DRIVETRAIN_REAR_RIGHT_PDP_CH  = 2;
	public static final int DRIVETRAIN_REAR_LEFT_PDP_CH   = 3;
	public static final int HOPPER_MOTOR_PDP_CH   = 8;
	public static final int CLIMBER_MOTOR1_PDP_CH  = 5;
	public static final int CLIMBER_MOTOR2_PDP_CH  = 6;
	public static final int INTAKE_MOTOR_PDP_CH   = 14;

	//CAN Device ID's
	public static final int SHOOTER_CAN_TALON_DEVICE_ID = 0;
	public static final int PDP_CAN_DEVICE_ID = 0;
	public static final int PCM_CAN_DEVICE_ID = 0;
	
	//Encoder Input Ports
	public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_A = 2;
	public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_B = 3;
	public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_A = 0;
	public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_B = 1;
	public static final int DRIVETRAIN_REAR_LEFT_ENCODER_A = 6;
	public static final int DRIVETRAIN_REAR_LEFT_ENCODER_B = 7;
	public static final int DRIVETRAIN_REAR_RIGHT_ENCODER_A = 4;
	public static final int DRIVETRAIN_REAR_RIGHT_ENCODER_B = 5;
	
	//Solenoid Ports
	public static final int GEAR_SOLENOID_PORT = 1; 
	public static final int INTAKE_EXTEND_SOLENOID_PORT = 2;
	//public static final int SW_TEST_BOARD_SOLENOID_PORT = 4; 
	
	//Analog Inputs
	public static final int AIR_PRESSURE_SENSOR_PORT = 1;
	
	//DIO Ports
	public static final int LED_RING_CONTROL_OUTPORT = 8;
	
	
	
	
	///////////////////////////////////////////////////////////////////////////
	// Vision Proc Camera & Vision Properties
	///////////////////////////////////////////////////////////////////////////
	/*
	//Axis M1013
	public static final int VISION_X_PIXELS = 800;
	public static final int VISION_Y_PIXELS = 600;
	public static final double CAMERA_FOV_X_DEG = 67; 
	public static final String VISION_PROC_CAMERA_URL = "http://10.17.36.10/mjpg/video.mjpg";
	*/
	
	
	//Axis M1011
	public static final int VISION_X_PIXELS = 640;
	public static final int VISION_Y_PIXELS = 480;
	public static final double CAMERA_FOV_X_DEG = 48; 
	public static final String VISION_PROC_CAMERA_URL = "http://10.17.36.10/mjpg/video.mjpg";
	
	
	/*
	//Microsoft Lifecam
	public static final int VISION_X_PIXELS = 800;
	public static final int VISION_Y_PIXELS = 600;
	public static final double CAMERA_FOV_X_DEG = 61; 
	public static final String VISION_PROC_CAMERA_URL = "http://10.17.36.9:8080/cam.mjpg";
	*/
	
	
	
	
	///////////////////////////////////////////////////////////////////////////
	// Drivetrain Physical
	///////////////////////////////////////////////////////////////////////////
	public static final double DRIVETRAIN_WHEELS_REV_PER_TICK = 1.0/2048.0; //2048 cycles per revolution (encoder libraries handle 4x decoding
	public static final double DRIVETRAIN_WHEELS_RADIUS_FT= 4.0/2.0/12.0; //4 inch diameter wheel, converted to radius in feet

	///////////////////////////////////////////////////////////////////////////
	// Climber
	///////////////////////////////////////////////////////////////////////////
	public static final double CLIMBER_MOTOR_MAX_ALLOWABLE_CURRENT_A = 80.0;
	public static final int CLIMBER_MOTOR_EXCESS_CURRENT_DBNC_LOOPS = 25;
	
	///////////////////////////////////////////////////////////////////////////
	// High Goal Vision Target
	///////////////////////////////////////////////////////////////////////////
	//Real
	//public static final double HIGH_GOAL_VISION_TARGET_HEIGHT_FT = 7.0 + (2.0/12.0); //7 ft 2 inches, per game manual 
	//private final double TGT_WIDTH_FT = 1.0 + (3.0/12.0); //1 ft 3 in, per game manual
	
	//Test 1/3 size Target
	public static final double HIGH_GOAL_VISION_TARGET_HEIGHT_FT = 4; //Where I happened to place it on Febrary 5th
	public static final double TGT_WIDTH_FT = (6.0+5.0/16.0)/12.0; //SW test target (6 and 5/16ths inches)
	
	
	
	
	///////////////////////////////////////////////////////////////////////////
	// Driver View Warning Indicator Settings
	///////////////////////////////////////////////////////////////////////////
	public static final double SYS_AIR_PRESSURE_CRITICAL_THRESH_PSI = 45.0;
	public static final int VISION_ALIGN_NOT_ALLOWED_BLINK_PERIOD_MS = 200;
	
	
	
	///////////////////////////////////////////////////////////////////////////
	// LED Strips
	///////////////////////////////////////////////////////////////////////////
	public static final int NUM_LEDS_TOTAL = 52;
	public static final Color CASSEROLE_RED    = new Color(255,0  ,0  ); //TBD from Zach
	public static final Color CASSEROLE_WHITE  = new Color(255,255,255); //TBD from Zach
	public static final Color CASSEROLE_YELLOW = new Color(255,200,50 ); //TBD from Zach
	
	
	///////////////////////////////////////////////////////////////////////////
	// Driver Camera gimbal mount position constants
	///////////////////////////////////////////////////////////////////////////
	
	//Large Metal Mount 
	/*
	private static final double GEAR_PAN_ANGLE = 180;
	private static final double GEAR_TILT_ANGLE = 30; //Bigger tilt makes the camera look up
	private static final double SHOOT_PAN_ANGLE = 90;
	private static final double SHOOT_TILT_ANGLE = 30;
	private static final double INTAKE_PAN_ANGLE = 0;
	private static final double INTAKE_TILT_ANGLE = 0;
	*/
	//Small 3d printed red mount
	public static final double GEAR_PAN_ANGLE = 180;
	public static final double GEAR_TILT_ANGLE = 90; //Bigger tilt makes the camera look up
	public static final double SHOOT_PAN_ANGLE = 90;
	public static final double SHOOT_TILT_ANGLE = 50;
	public static final double INTAKE_PAN_ANGLE = 0;
	public static final double INTAKE_TILT_ANGLE = 100;
	
}
