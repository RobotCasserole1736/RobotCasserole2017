package org.usfirst.frc.team1736.robot;

public class RobotConstants {
	
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
	
	
	///////////////////////////////////////////////////////////////////////////
	// Physical Constants
	///////////////////////////////////////////////////////////////////////////
	//Camera & Vision Properties
	public static final int VISION_X_PIXELS = 640;
	public static final int VISION_Y_PIXELS = 480;
	
	//Drivetrain
	public static final double DRIVETRAIN_WHEELS_REV_PER_TICK = 1.0/2048.0; //2048 cycles per revolution (encoder libraries handle 4x decoding
	public static final double DRIVETRAIN_WHEELS_RADIUS_FT= 4.0/2.0/12.0; //4 inch diameter wheel, converted to radius in feet

	//Playing Field
	public static final double HIGH_GOAL_VISION_TARGET_HEIGHT_FT = 6;
}
