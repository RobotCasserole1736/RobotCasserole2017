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
		Shooter_ff_Gain = new Calibration("Shooter_ff_Gain",0);
		Shooter_P_Gain = new Calibration("Shooter_P_Gain",0);
		Shooter_I_Gain = new Calibration("Shooter_I_Gain",0);
		Shooter_D_Gain = new Calibration("Shooter_D_Gain",0);
		ErrorRange = new Calibration("Shooter_Error_Limit_RPM",0);

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
		if(Shooter_ff_Gain.is_updated){
			TallonFlame.setF(Shooter_ff_Gain.get()); 
			Shooter_ff_Gain.is_updated = false;
		}
		
		if(Shooter_P_Gain.is_updated){
			TallonFlame.setP(Shooter_P_Gain.get()); 
			Shooter_P_Gain.is_updated = false;
		}
		
		if(Shooter_I_Gain.is_updated){
			TallonFlame.setI(Shooter_I_Gain.get()); 
			Shooter_I_Gain.is_updated = false;
		}
		
		if(Shooter_D_Gain.is_updated){
			TallonFlame.setD(Shooter_D_Gain.get()); 
			Shooter_D_Gain.is_updated = false;
		}
		
	}
	
	
	public void update(){
		TallonFlame.set(RobotState.shooterDesiredVelocity_rpm); // set what speed the wheel should be running at 
		RobotState.shooterActualVelocity_rpm = TallonFlame.getSpeed();
		double Error = Math.abs(RobotState.shooterDesiredVelocity_rpm -RobotState.shooterActualVelocity_rpm);
		if (Error > ErrorRange.get()){
			RobotState.shooterVelocityOk = false;
		}else{
			RobotState.shooterVelocityOk = true;
		}
	
	}
	
}   