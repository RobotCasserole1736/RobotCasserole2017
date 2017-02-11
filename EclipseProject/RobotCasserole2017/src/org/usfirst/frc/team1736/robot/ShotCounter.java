package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.SignalMath.AveragingFilter;
import org.usfirst.frc.team1736.lib.SignalMath.DerivativeCalculator;

public class ShotCounter {
	private static ShotCounter shotCounter;
	private DerivativeCalculator accelCalc;
	private DerivativeCalculator jerkCalc;
	private AveragingFilter jerkFilter;
	private final double jerkThresh = 70000;
	private int currCount;
	private boolean aboveThresh;
	
	
	public static synchronized ShotCounter getInstance()
	{
		if(shotCounter == null)
			shotCounter = new ShotCounter();
		return shotCounter;
	}
	
	private ShotCounter() {
		accelCalc = new DerivativeCalculator();
		jerkCalc = new DerivativeCalculator();
		jerkFilter = new AveragingFilter(3, 0);
		currCount = 0;
		aboveThresh = false;
	}
	
	public void update(){
		double rpm = ShooterWheelCtrl.getInstance().getShooterActualVelocityRPM();
		if(rpm > ShooterWheelCtrl.getInstance().getShooterDesiredRPM()*0.5){ //Only run logic if shooter wheel is above half speed
			ShooterWheelCtrl.getInstance().getShooterActualVelocityRPM();
			double rpmps = accelCalc.calcDeriv(rpm);
			double jerk = jerkCalc.calcDeriv(rpmps); 
			double filtJerk = jerkFilter.filter(jerk);
			if(filtJerk > jerkThresh && aboveThresh == false){
				aboveThresh = true;
			}
			if(filtJerk < jerkThresh && aboveThresh == true){
				currCount = currCount + 1;
				aboveThresh = false;
			}
		}
	}
	public int getCurrCount(){
		return currCount;
	}
	
	public double getCurrCountLog(){
		return currCount;
	}

}
