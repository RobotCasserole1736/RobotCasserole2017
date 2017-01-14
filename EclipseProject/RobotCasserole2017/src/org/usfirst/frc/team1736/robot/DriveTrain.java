package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;

public class DriveTrain{

	RobotDrive myDrive;
	Joystick moveStick, rotateStick;
	
	public void RobotInit() {
		myDrive = new RobotDrive(0, 1, 2, 3);
		moveStick = new Joystick(0);
		rotateStick = new Joystick(1);
	}
	
	public void autonomous() {
	}
	
	public void OperatorControl() {
		while (OperatorControl() && isEnabled()) {
			myDrive.mecanumDrive_Cartesian(moveStick.getY(), moveStick.getX(), rotateStick.getX(), 0);
			Timer.delay(0.02);
		}
	}
}