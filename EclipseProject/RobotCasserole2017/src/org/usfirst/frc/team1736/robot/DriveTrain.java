package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import org.usfirst.frc.team1736.robot.RobotState;
import org.usfirst.frc.team1736.lib.HAL.Xbox360Controller;
public class DriveTrain{
	
	
	Xbox360Controller driverCTRL;
	Xbox360Controller operatorCTRL;
	
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
	//plug 3DOF into the method autonomous
/*	public void autonomous() {
		myDrive.mecanumDrive_Cartesian(driverFwdRevCmd, driverStrafeCmd, driverRotateCmd, 0);
		Timer.delay(0.02);
	}
	*/
	public void OperatorControl() {
			double driverFwdRevCmd = driverCTRL.LStick_Y();
			double driverStrafeCmd = driverCTRL.LStick_X();
			double driverRotateCmd = driverCTRL.RStick_X();
			myDrive.mecanumDrive_Cartesian(driverFwdRevCmd, driverStrafeCmd, driverRotateCmd, 0);
			Timer.delay(0.02);
	}
}