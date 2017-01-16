package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

public class DriveTrain{

	public static Object myRobot;
	RobotDrive myDrive;
	Joystick moveStick, rotateStick;
	
	public void RobotInit() {
		Victor frontLeft = new Victor(1);
    	Victor frontRight = new Victor(2);
    	Victor rearLeft = new Victor (3);
    	Victor rearRight = new Victor (4);
    	
    	
    	myDrive = new RobotDrive(frontLeft, frontRight, rearLeft, rearRight);
		myDrive = new RobotDrive(0, 1, 2, 3);
		moveStick = new Joystick(0);
		rotateStick = new Joystick(1);
	}
	
	public void autonomous() {
	}
	
	public void OperatorControl() {
	//		myDrive.mecanumDrive_Cartesian(moveStick.getY(), moveStick.getX(), rotateStick.getX(), 0);
			Timer.delay(0.02);
	}
}