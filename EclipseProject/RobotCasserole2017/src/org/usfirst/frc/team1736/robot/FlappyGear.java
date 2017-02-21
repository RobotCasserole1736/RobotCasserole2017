package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Servo;

public class FlappyGear {
	
	private static FlappyGear flappyGear = null;

	int FLAP_DOWN_DEG = 190;
	int FLAP_UP_DEG = 65;
	Servo servo;
	
	public static synchronized FlappyGear getInstance(){
		if(flappyGear == null)
			flappyGear = new FlappyGear();
		return flappyGear;
	}
	
	private FlappyGear(){
	 servo = new Servo(RobotConstants.GEAR_FLAP_SERVO_PWM_PORT);	
	}
	
	public void update(){
	
		servo.setAngle( (FLAP_DOWN_DEG - FLAP_UP_DEG)* OperatorController.getInstance().getGearFlapCommand() + FLAP_UP_DEG);
	}
	
	public void setAngle(int angle)
	{
		servo.setAngle(angle);
	}
	
	
}

