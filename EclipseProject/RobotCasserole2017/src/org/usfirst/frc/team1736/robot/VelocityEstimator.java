package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.SignalMath.AveragingFilter;
import org.usfirst.frc.team1736.lib.SignalMath.IntegralCalculator;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

public class VelocityEstimator {
	
	//Uses the roboRIO's internal accelerometer to make a guess at the robot's velocity
	//Estimated by integrating a noisy signal, so might be kinda wrong. This is totally experimental
	
	//Assumes the RIO is mounted flat in the belly pan of the robot, with the +X accelerometer direction facing
	// the front of the robot, and the +Y accelerometer direction facing the left side.
	
	IntegralCalculator fwdRevVelIntegrator;
	IntegralCalculator strafeVelIntegrator;
	
	AveragingFilter accelXAvgOffsetCalc;
	AveragingFilter accelYAvgOffsetCalc;
	
	BuiltInAccelerometer accel;
	
	double estFwdRevVel;
	double estStrafeVel;
	
	final double FT_PER_SEC_2_PER_G = 32.174; //Conversion factor for the gravitational constant (g) to ft/sec2
	
	boolean robotMovingPrev = false;
	
	double accelXOffset = 0;
	double accelYOffset = 0;
	
	public VelocityEstimator(){
		accel = new BuiltInAccelerometer(Accelerometer.Range.k2G);
		
		fwdRevVelIntegrator = new IntegralCalculator(3); //use 3rd order estimation
		strafeVelIntegrator = new IntegralCalculator(3);
		
		accelXAvgOffsetCalc = new AveragingFilter(30, 0);
		accelYAvgOffsetCalc = new AveragingFilter(30, 0);
		
	}
	
	public void update(){
		boolean robotMovingNow = isRobotMoving();
		double accelXThisLoop = accel.getX();
		double accelYThisLoop = accel.getY();
		//Fundamentally, we estimate in two states. We use the encoders to see whether the robot is stationary or not.
		//If the robot isn't stationary, we can use the integrated acceleration to figure out how fast we're going.
		//But, if the encoders say we're stationary, we'll also estimate we aren't moving, and try to track the offset
		// of the accelerometer (RIO not level, noise, etc).
		//Every time we transition from not-moving to moving, we reset the integration
		//Every time we transition from moving to not-moving, we reset the average offset.
		
		if(robotMovingNow == true & robotMovingPrev == false){
			//stationary to moving transition
			fwdRevVelIntegrator.resetIntegral();
			strafeVelIntegrator.resetIntegral();
			
			accelXOffset = accelXAvgOffsetCalc.filter(accelXThisLoop);
			accelYOffset = accelYAvgOffsetCalc.filter(accelYThisLoop);


		} else if(robotMovingNow == false & robotMovingPrev == true){
			//moving to stationary transition
			accelXAvgOffsetCalc.reset();
			accelYAvgOffsetCalc.reset();
			
		}
		
		
		if(!robotMovingNow){
			//Encoders say the robot shouldn't be moving at all. We'll estimate 0 motion then.
			//Additionally, we'll try to calculate the average offset of the gyro
			
			accelXAvgOffsetCalc.filter(accelXThisLoop);
			accelYAvgOffsetCalc.filter(accelYThisLoop);
			
			estFwdRevVel = 0.0;
			estStrafeVel = 0.0;

		} else {
			estFwdRevVel = fwdRevVelIntegrator.calcIntegral(accelXThisLoop*FT_PER_SEC_2_PER_G);
			estStrafeVel = strafeVelIntegrator.calcIntegral(accelYThisLoop*FT_PER_SEC_2_PER_G*-1.0);
		}
		
		//Update State
		robotMovingPrev = robotMovingNow;
	}
	
	//True if encoders say the robot should be moving, false if not.
	private boolean isRobotMoving(){
		return (Math.abs(DriveTrain.getInstance().getFrontLeftWheelSpeedRPM()) >= 1.0 &
				Math.abs(DriveTrain.getInstance().getFrontRightWheelSpeedRPM()) >= 1.0 &
				Math.abs(DriveTrain.getInstance().getRearLeftWheelSpeedRPM()) >= 1.0 &
				Math.abs(DriveTrain.getInstance().getRearRightWheelSpeedRPM()) >= 1.0); 
	}
	
	public double getEstFwdRevVel_ftpers(){
		return estFwdRevVel;
	}
	
	public double getEstStrafeVel_ftpers(){
		return estStrafeVel;
	}

}
