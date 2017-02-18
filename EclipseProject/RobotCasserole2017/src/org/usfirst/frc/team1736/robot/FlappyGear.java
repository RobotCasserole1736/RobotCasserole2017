package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Servo;

public class FlappyGear {

	int FLAP_UP_DEG = 90;
	int FLAP_DOWN_DEG = 0;
	Servo servo;
	public FlappyGear(){
	 servo = new Servo(RobotConstants.GEAR_FLAP_SERVO_PWM_PORT);	
	}
	
	public void update(){
	
		servo.setAngle( (FLAP_UP_DEG - FLAP_DOWN_DEG)* OperatorController.getInstance().getGearFlapCommand() + FLAP_DOWN_DEG);
	}
	
	
}

