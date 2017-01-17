package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import org.usfirst.frc.team1736.robot.RobotState;
import org.usfirst.frc.team1736.lib.HAL.Xbox360Controller;
public class DriveTrain{
	
	
	public static final int DRIVER_FRONT_RIGHT_MOTOR = 0;
	public static final int DRIVER_FRONT_LEFT_MOTOR = 1;
	public static final int DRIVER_REAR_LEFT_MOTOR = 2;
	public static final int DRIVER_REAR_RIGHT_MOTOR = 3;
	
	Xbox360Controller driverCTRL;
	Xbox360Controller operatorCTRL;
	
	public static Object myRobot;
	RobotDrive myDrive;
	Joystick moveStick, rotateStick;
	Victor frontLeft;
	Victor frontRight;
	Victor rearLeft;
	Victor rearRight;
	
	public void RobotInit() {
		frontLeft = new Victor(DRIVER_FRONT_LEFT_MOTOR);
    	frontRight = new Victor(DRIVER_FRONT_RIGHT_MOTOR);
    	rearLeft = new Victor (DRIVER_REAR_LEFT_MOTOR);
    	rearRight = new Victor (DRIVER_REAR_RIGHT_MOTOR);
    	
    	
    	myDrive = new RobotDrive(frontLeft, frontRight, rearLeft, rearRight);
	}
	//plug 3DOF into the method autonomous
/*	public void autonomous() {
		myDrive.mecanumDrive_Cartesian(driverFwdRevCmd, driverStrafeCmd, driverRotateCmd, 0);
		Timer.delay(0.02);
	}
	*/
	public void OperatorControl() {
			myDrive.mecanumDrive_Cartesian(RobotState.driverFwdRevCmd, RobotState.driverStrafeCmd, RobotState.driverRotateCmd, 0);

	
	}

	public double driverFL() {
		return frontLeft.get();
	}
	
	public double driverFR() {
		return frontRight.get();
	}
	
	public double driverRL() {
		return rearLeft.get();
	}

	public double driverRR() {
		return rearRight.get();
	}



}