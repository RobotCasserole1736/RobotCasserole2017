package org.usfirst.frc.team1736.robot;
	
import org.usfirst.frc.team1736.lib.Calibration.Calibration;
	
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
public class ShooterWheelCtrl {
	
	Calibration Shooter_ff_Gain;
	Calibration Shooter_P_Gain;
	Calibration Shooter_I_Gain;
	Calibration Shooter_D_Gain;
	Calibration ErrorRange;
	CANTalon shooterTalon;
	
	
	public ShooterWheelCtrl(){
		shooterTalon = new CANTalon(RobotConstants.SHOOTER_CAN_TALON_DEVICE_ID);
		Shooter_ff_Gain = new Calibration("Shooter FeedFwd Gain",0);
		Shooter_P_Gain = new Calibration("Shooter P Gain",0.001);
		Shooter_I_Gain = new Calibration("Shooter I Gain",0);
		Shooter_D_Gain = new Calibration("Shooter D Gain",0);
		ErrorRange = new Calibration("Shooter Error Limit RPM",10,10,1000);

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
		shooterTalon.set(RobotState.shooterDesiredVelocity_rpm); // set what speed the wheel should be running at 
		RobotState.shooterActualVelocity_rpm = shooterTalon.getSpeed();
		RobotState.shooterMotorCmd = shooterTalon.get();
		double Error = Math.abs(RobotState.shooterDesiredVelocity_rpm -RobotState.shooterActualVelocity_rpm);
		if (Error > ErrorRange.get()){
			RobotState.shooterVelocityOk = false;
		}else{
			RobotState.shooterVelocityOk = true;
		}
	
	}
	
}