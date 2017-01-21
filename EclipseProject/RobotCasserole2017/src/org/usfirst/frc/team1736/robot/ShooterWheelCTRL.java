package org.usfirst.frc.team1736.robot;
	
import org.usfirst.frc.team1736.lib.Calibration.Calibration;
	
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
public class ShooterWheelCTRL {
	
	Calibration Shooter_ff_Gain;
	Calibration Shooter_P_Gain;
	Calibration Shooter_I_Gain;
	Calibration Shooter_D_Gain;
	Calibration ErrorRange;
	CANTalon TallonFlame;
	
	
	public ShooterWheelCTRL(){
		TallonFlame = new CANTalon(RobotIOMap.SHOOTER_CAN_TALON_DEVICE_ID);
		Shooter_ff_Gain = new Calibration("Shooter FeedFwd Gain",0);
		Shooter_P_Gain = new Calibration("Shooter P Gain",0.001);
		Shooter_I_Gain = new Calibration("Shooter I Gain",0);
		Shooter_D_Gain = new Calibration("Shooter D Gain",0);
		ErrorRange = new Calibration("Shooter Error Limit RPM",10,10,1000);

		TallonFlame.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative); //Tells the SRX an encoder is attached to its input.
		TallonFlame.setProfile(0); //Select slot 0 for PID gains
		TallonFlame.changeControlMode(TalonControlMode.Speed); //Set that a PID algorithm should be used to control the output
		
		TallonFlame.setF(Shooter_ff_Gain.get()); //Set the PID algorithm gains based on calibration values
		TallonFlame.setP(Shooter_P_Gain.get());
		TallonFlame.setI(Shooter_I_Gain.get()); 
		TallonFlame.setD(Shooter_D_Gain.get());

	}
	
	/**
	 * Update the gains if needed
	 */
	
	public void updateGains(){
		//Set the PID algorithm gains based on calibration values
		if(Shooter_ff_Gain.isChanged()){
			TallonFlame.setF(Shooter_ff_Gain.get()); 
			Shooter_ff_Gain.acknowledgeValUpdate();
		}
		
		if(Shooter_P_Gain.isChanged()){
			TallonFlame.setP(Shooter_P_Gain.get()); 
			Shooter_P_Gain.acknowledgeValUpdate();
		}
		
		if(Shooter_I_Gain.isChanged()){
			TallonFlame.setI(Shooter_I_Gain.get()); 
			Shooter_I_Gain.acknowledgeValUpdate();
		}
		
		if(Shooter_D_Gain.isChanged()){
			TallonFlame.setD(Shooter_D_Gain.get()); 
			Shooter_D_Gain.acknowledgeValUpdate();
		}
		
	}
	
	
	public void update(){
		TallonFlame.set(RobotState.shooterDesiredVelocity_rpm); // set what speed the wheel should be running at 
		RobotState.shooterActualVelocity_rpm = TallonFlame.getSpeed();
		RobotState.shooterMotorCmd = TallonFlame.get();
		double Error = Math.abs(RobotState.shooterDesiredVelocity_rpm -RobotState.shooterActualVelocity_rpm);
		if (Error > ErrorRange.get()){
			RobotState.shooterVelocityOk = false;
		}else{
			RobotState.shooterVelocityOk = true;
		}
	
	}
	
}