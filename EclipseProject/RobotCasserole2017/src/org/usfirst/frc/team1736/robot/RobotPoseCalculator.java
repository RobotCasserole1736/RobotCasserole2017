package org.usfirst.frc.team1736.robot;


public class RobotPoseCalculator {
	
	public RobotPoseCalculator(){
		return;
	}
	
	public void update(){
		//The calculations referenced herein are derived from this helpful paper:
		// http://www.academia.edu/4557426/KINEMATICS_MODELLING_OF_MECANUM_WHEELED_MOBILE_PLATFORM
		
		double WheelSpeedOno;
		double WheelSpeedDos;
		double WheelSpeedTres;
		double WheelSpeedCuatro;
		double Vx;
		double Vy;
		double netSpeed;
		
		///////////////////////////////////////////////////////////////////////////
		// Speed calculations
		///////////////////////////////////////////////////////////////////////////
		
		//Calculate wheel speeds in radians per second
		WheelSpeedOno =RobotState.frontLeftWheelVelocity_rpm * 2.0 * Math.PI / 60; 
		WheelSpeedDos =RobotState.frontRightWheelVelocity_rpm * 2.0 * Math.PI / 60;
		WheelSpeedTres =RobotState.rearLeftWheelVelocity_rpm * 2.0 * Math.PI / 60;
		WheelSpeedCuatro =RobotState.rearRightWheelVelocity_rpm * 2.0 * Math.PI / 60;
	
		//Calculate translational velocity x/y components via inverse mechanum kinematic equations
		Vx = (WheelSpeedOno + WheelSpeedDos + WheelSpeedTres + WheelSpeedCuatro) * RobotConstants.DRIVETRAIN_WHEELS_RADIUS_FT / 4;
		Vy  = (WheelSpeedOno - WheelSpeedDos + WheelSpeedTres - WheelSpeedCuatro) * RobotConstants.DRIVETRAIN_WHEELS_RADIUS_FT / 4;
	
		//Calculate net speed vector with pythagorean theorem
		netSpeed = Math.sqrt(Vx*Vx+Vy*Vy);
		
		//Store results into state variables
		RobotState.robotNetSpeed_ftpers = netSpeed;
		RobotState.robotFwdRevVel_ftpers = Vx;
		RobotState.robotStrafeVel_ftpers = Vy;	
		
		///////////////////////////////////////////////////////////////////////////
		// Distance calculations - similar to above
		///////////////////////////////////////////////////////////////////////////
		
		RobotState.robotFwdRevDist_ft = (RobotState.frontLeftWheelDistance_ft + 
				                         RobotState.frontRightWheelDistance_ft + 
				                         RobotState.rearLeftWheelDistance_ft + 
				                         RobotState.rearRightWheelDistance_ft) * RobotConstants.DRIVETRAIN_WHEELS_RADIUS_FT / 4;
		RobotState.robotStrafeDist_ft  = (RobotState.frontLeftWheelDistance_ft - 
										 RobotState.frontRightWheelDistance_ft +
										 RobotState.rearLeftWheelDistance_ft - 
										 RobotState.rearRightWheelDistance_ft) * RobotConstants.DRIVETRAIN_WHEELS_RADIUS_FT / 4;
		
		RobotState.robotNetDistance_ft =  Math.sqrt(Math.pow(RobotState.robotStrafeDist_ft, 2)+Math.pow(RobotState.robotFwdRevDist_ft, 2));
	}
}
