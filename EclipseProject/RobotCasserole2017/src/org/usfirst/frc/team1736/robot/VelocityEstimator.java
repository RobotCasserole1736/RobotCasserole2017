package org.usfirst.frc.team1736.robot;

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
	
	BuiltInAccelerometer accel;
	
	double estFwdRevVel;
	double estStrafeVel;
	
	final double FT_PER_SEC_2_PER_G = 32.174; //Conversion factor for the gravitational constant (g) to 
	
	public VelocityEstimator(){
		accel = new BuiltInAccelerometer(Accelerometer.Range.k2G);
		
		fwdRevVelIntegrator = new IntegralCalculator(3); //use 3rd order estimation
		strafeVelIntegrator = new IntegralCalculator(3);
	}
	
	public void update(){
		estFwdRevVel = fwdRevVelIntegrator.calcIntegral(accel.getX()*FT_PER_SEC_2_PER_G);
		estStrafeVel = fwdRevVelIntegrator.calcIntegral(accel.getY()*FT_PER_SEC_2_PER_G*-1.0);
	}
	
	public double getEstFwdRevVel_ftpers(){
		return estFwdRevVel;
	}
	
	public double getEstStrafeVel_ftpers(){
		return estStrafeVel;
	}

}
