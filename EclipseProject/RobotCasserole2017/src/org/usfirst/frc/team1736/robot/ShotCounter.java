package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.SignalMath.DerivativeCalculator;

public class ShotCounter {
	private static ShotCounter shotCounter;
	private DerivativeCalculator accelCalc;
	private DerivativeCalculator jerkCalc;
	private final double jerkThresh = 100;
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
		currCount = 0;
		aboveThresh = false;
	}
	
	public void update(){
		ShooterWheelCtrl.getInstance().getShooterActualVelocityRPM();
		double rpm = ShooterWheelCtrl.getInstance().getShooterActualVelocityRPM();
		double rpmps = accelCalc.calcDeriv(rpm);
		double jerk = jerkCalc.calcDeriv(rpmps);
		if(jerk > jerkThresh && aboveThresh == false){
			aboveThresh = true;
		}
		if(jerk < jerkThresh && aboveThresh == true){
			currCount = currCount + 1;
			aboveThresh = false;
		}
	}
	public int getCurrCount(){
		return currCount;
	}
	
	public double getCurrCountLog(){
		return currCount;
	}

}
