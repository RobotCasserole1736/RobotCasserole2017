package org.usfirst.frc.team1736.robot;


public class RobotPoseCalculator {
	
	public RobotPoseCalculator(){
		return;
	}
	
	public void update(){
		double WheelSpeedOno;
		double WheelSpeedDos;
		double WheelSpeedTres;
		double WheelSpeedCuatro;
		double Vx;
		double Vy;
		double netSpeed;
		
		//Calculate wheel speeds in radians per second
		WheelSpeedOno =RobotState.frontLeftWheelVelocity_rpm * 2.0 * Math.PI / 60; 
		WheelSpeedDos =RobotState.frontRightWheelVelocity_rpm * 2.0 * Math.PI / 60;
		WheelSpeedTres =RobotState.rearLeftWheelVelocity_rpm * 2.0 * Math.PI / 60;
		WheelSpeedCuatro =RobotState.rearRightWheelVelocity_rpm * 2.0 * Math.PI / 60;
	
		//Calculate translational velocity x/y components via inverse mechanum kinematic equations
		Vx = (WheelSpeedOno + WheelSpeedDos + WheelSpeedTres + WheelSpeedCuatro) * 0.17 / 4;
		Vy  = (WheelSpeedOno - WheelSpeedDos + WheelSpeedTres - WheelSpeedCuatro) * 0.17 / 4;
	
		//Calculate net speed vector with pythagorean theorem
		netSpeed = Math.sqrt(Vx*Vx+Vy*Vy);
		
		//Store results into state variables
		RobotState.robotNetSpeed_ftpers = netSpeed;
		RobotState.robotFwdRevVel_ftpers = Vx;
		RobotState.robotStrafeVel_ftpers = Vy;		
	}
}
