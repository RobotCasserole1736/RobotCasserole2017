/**
 * 
 */
package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Servo;

/**
 * @author gerthcm
 *
 */
public class CameraServoMount {
	
	//State variables
	public double cur_pan_angle;
	public double cur_tilt_angle;
	public CamPos curCamPos;
	
	//Startup conditions
	private static final CamPos startupPos = CamPos.SHOOT;
	
	//Position constants (in degrees)
	//Bigger tilt makes the camera look up
	private static final double GEAR_PAN_ANGLE = 180;
	private static final double GEAR_TILT_ANGLE = 30;
	
	private static final double SHOOT_PAN_ANGLE = 90;
	private static final double SHOOT_TILT_ANGLE = 30;
	
	private static final double INTAKE_PAN_ANGLE = 0;
	private static final double INTAKE_TILT_ANGLE = 0;
	

	
	//Servo objects for mount servos
	private Servo pan_servo;
	private Servo tilt_servo;
	
	/** 
	 * Constructor - initializes all the objects for a camera servo mount. Takes nothing, returns nothing.
	 */
	CameraServoMount(){
		pan_servo = new Servo(RobotConstants.CAMERA_PAN_SERVO_PWM_PORT);
		tilt_servo = new Servo(RobotConstants.CAMERA_TILT_SERVO_PWM_PORT);
		setCameraPos(startupPos);
		
	}
	
	public void update(){
		if(RobotState.gearCamAlign){
			setCameraPos(CamPos.GEAR);
		} else if(RobotState.intakeCamAlign) {
			setCameraPos(CamPos.INTAKE);
		} else if(RobotState.shooterCamAlign){
			setCameraPos(CamPos.SHOOT);
		}
	}
	
	/**
	 * Commands the servos to the right spots based on the value of camera position in
	 * @param in
	 */
	
	public void setCameraPos(CamPos in){
		resolveCamPos(in);
		pan_servo.setAngle(cur_pan_angle);
		tilt_servo.setAngle(cur_tilt_angle);
		
		
	}
	
	/**
	 * Sets the pan and tilt internal variables per the
	 * position specified in the input argument.
	 * @param in - position to set the camera to.
	 */
	private void resolveCamPos(CamPos in){
		curCamPos = in;
		
		switch(in){
		case INTAKE:
			cur_pan_angle =  INTAKE_PAN_ANGLE;
			cur_tilt_angle = INTAKE_TILT_ANGLE;
			break;			
		case SHOOT:
			cur_pan_angle =  SHOOT_PAN_ANGLE;
			cur_tilt_angle = SHOOT_TILT_ANGLE;
			break;
		case GEAR:
			cur_pan_angle =  GEAR_PAN_ANGLE;
			cur_tilt_angle = GEAR_TILT_ANGLE;
			break;
		default:
			System.out.println("Warning - commanded camera position " + in.name() + " is not recognized!");
			break;
		}
		
	}

}


