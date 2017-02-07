package org.usfirst.frc.team1736.robot;


public class RobotPoseCalculator {
	
	private static RobotPoseCalculator poseCalculator = null;
	
	private double netSpeed = 0;
	private double fwdRevVel = 0;
	private double strafeVel = 0;
	private double fwdRevDist = 0;
	private double strafeDist = 0;
	private double netDist = 0;
	
	public static synchronized RobotPoseCalculator getInstance()
	{
		if(poseCalculator == null)
			poseCalculator = new RobotPoseCalculator();
		return poseCalculator;
	}
	
	private RobotPoseCalculator(){
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
		DriveTrain dt = DriveTrain.getInstance();
		
		///////////////////////////////////////////////////////////////////////////
		// Speed calculations
		///////////////////////////////////////////////////////////////////////////
		
		//Calculate wheel speeds in radians per second
		WheelSpeedOno = dt.getFrontLeftWheelSpeedRPM() * 2.0 * Math.PI / 60; 
		WheelSpeedDos = dt.getFrontRightWheelSpeedRPM() * 2.0 * Math.PI / 60;
		WheelSpeedTres = dt.getRearLeftWheelSpeedRPM() * 2.0 * Math.PI / 60;
		WheelSpeedCuatro = dt.getRearRightWheelSpeedRPM() * 2.0 * Math.PI / 60;
	
		//Calculate translational velocity x/y components via inverse mechanum kinematic equations
		Vx = (WheelSpeedOno + WheelSpeedDos + WheelSpeedTres + WheelSpeedCuatro) * RobotConstants.DRIVETRAIN_WHEELS_RADIUS_FT / 4;
		Vy  = (WheelSpeedOno - WheelSpeedDos + WheelSpeedTres - WheelSpeedCuatro) * RobotConstants.DRIVETRAIN_WHEELS_RADIUS_FT / 4;
	
		//Calculate net speed vector with pythagorean theorem
		netSpeed = Math.sqrt(Vx*Vx+Vy*Vy);
		
		//Store results into state variables
		fwdRevVel = Vx;
		strafeVel = Vy;	
		
		///////////////////////////////////////////////////////////////////////////
		// Distance calculations - similar to above
		///////////////////////////////////////////////////////////////////////////
		
		fwdRevDist = (dt.getFrontLeftWheelDistanceFt() + 
				                         dt.getFrontRightWheelDistanceFt() + 
				                         dt.getRearLeftWheelDistanceFt() + 
				                         dt.getRearRightWheelDistanceFt()) / 4.0;
		strafeDist = (dt.getFrontLeftWheelDistanceFt() - 
										 dt.getFrontRightWheelDistanceFt() +
										 dt.getRearLeftWheelDistanceFt() - 
										 dt.getRearRightWheelDistanceFt()) / 4.0;
		
		netDist +=  netSpeed*0.02; //meh, just a guess at sample time. This isn't used for anything now that I know of.
	}
	
	public double getNetSpeedFtPerS()
	{
		return netSpeed;
	}
	
	public double getFwdRevVelFtPerS()
	{
		return fwdRevVel;
	}
	
	public double getStrafeVelFtPerS()
	{
		return strafeVel;
	}
	
	public double getFwdRevDistFt()
	{
		return fwdRevDist;
	}
	
	public double getStrafeDistFt()
	{
		return strafeDist;
	}
	
	public double getNetDistFt()
	{
		return netDist;
	}
}
