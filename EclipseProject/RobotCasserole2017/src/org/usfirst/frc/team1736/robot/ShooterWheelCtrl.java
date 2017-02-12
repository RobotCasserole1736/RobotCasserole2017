package org.usfirst.frc.team1736.robot;

/*
 *******************************************************************************************
 * Copyright (C) 2017 FRC Team 1736 Robot Casserole - www.robotcasserole.org
 *******************************************************************************************
 *
 * This software is released under the MIT Licence - see the license.txt
 *  file in the root of this repo.
 *
 * Non-legally-binding statement from Team 1736:
 *  Thank you for taking the time to read through our software! We hope you
 *   find it educational and informative! 
 *  Please feel free to snag our software for your own use in whatever project
 *   you have going on right now! We'd love to be able to help out! Shoot us 
 *   any questions you may have, all our contact info should be on our website
 *   (listed above).
 *  If you happen to end up using our software to make money, that is wonderful!
 *   Robot Casserole is always looking for more sponsors, so we'd be very appreciative
 *   if you would consider donating to our club to help further STEM education.
 */
	
import org.usfirst.frc.team1736.lib.Calibration.Calibration;
	
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

public class ShooterWheelCtrl {
	
	private static ShooterWheelCtrl wheelCtrl = null;
	
	Calibration Shooter_ff_Gain;
	Calibration Shooter_P_Gain;
	Calibration Shooter_I_Gain;
	Calibration Shooter_D_Gain;
	Calibration ErrorRange;
	private CANTalon shooterTalon;
	
	private double desiredVelocity = 0;
	private double actualVelocity = 0;
	private double motorCmd = 0;
	private boolean isVelocityOk = false;
	
	
	public static synchronized ShooterWheelCtrl getInstance()
	{
		if(wheelCtrl == null)
			wheelCtrl = new ShooterWheelCtrl();
		return wheelCtrl;
	}
	
	private ShooterWheelCtrl(){
		shooterTalon = new CANTalon(RobotConstants.SHOOTER_CAN_TALON_DEVICE_ID);
		Shooter_ff_Gain = new Calibration("Shooter FeedFwd Gain",1.0/4000.0);
		Shooter_P_Gain = new Calibration("Shooter P Gain",0.1/400.0);
		Shooter_I_Gain = new Calibration("Shooter I Gain",0);
		Shooter_D_Gain = new Calibration("Shooter D Gain",0);
		ErrorRange = new Calibration("Shooter Error Limit RPM",100,10,1000);

		shooterTalon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative); //Tells the SRX an encoder is attached to its input.
		shooterTalon.setProfile(0); //Select slot 0 for PID gains
		shooterTalon.changeControlMode(TalonControlMode.Speed); //Set that a PID algorithm should be used to control the output
		
		shooterTalon.setF(Shooter_ff_Gain.get()); //Set the PID algorithm gains based on calibration values
		shooterTalon.setP(Shooter_P_Gain.get());
		shooterTalon.setI(Shooter_I_Gain.get()); 
		shooterTalon.setD(Shooter_D_Gain.get());

	}
	
	/**
	 * Update the gains if needed
	 */
	
	public void updateGains(){
		//Set the PID algorithm gains based on calibration values
		if(Shooter_ff_Gain.isChanged()){
			shooterTalon.setF(Shooter_ff_Gain.get()); 
			Shooter_ff_Gain.acknowledgeValUpdate();
		}
		
		if(Shooter_P_Gain.isChanged()){
			shooterTalon.setP(Shooter_P_Gain.get()); 
			Shooter_P_Gain.acknowledgeValUpdate();
		}
		
		if(Shooter_I_Gain.isChanged()){
			shooterTalon.setI(Shooter_I_Gain.get()); 
			Shooter_I_Gain.acknowledgeValUpdate();
		}
		
		if(Shooter_D_Gain.isChanged()){
			shooterTalon.setD(Shooter_D_Gain.get()); 
			Shooter_D_Gain.acknowledgeValUpdate();
		}
		
	}
	
	
	public void update(){
		shooterTalon.set(desiredVelocity); // set what speed the wheel should be running at 
		actualVelocity = shooterTalon.getSpeed();
		motorCmd = shooterTalon.get();
		double Error = Math.abs(desiredVelocity - actualVelocity);
		if (Error > ErrorRange.get()){
			isVelocityOk = false;
		}else{
			isVelocityOk = true;
		}
	
	}
	
	public void setShooterDesiredRPM(double rpm)
	{
		desiredVelocity = rpm;
	}
	
	public double getShooterDesiredRPM()
	{
		return desiredVelocity;
	}
	
	public boolean getShooterVelocityOK()
	{
		return isVelocityOk;
	}
	
	public double getShooterActualVelocityRPM()
	{
		return actualVelocity;
	}
	
	public double getShooterMotorCmd()
	{
		return motorCmd;
	}
	
	public double getOutputCurrent()
	{
		return shooterTalon.getOutputCurrent();
	}
	
}