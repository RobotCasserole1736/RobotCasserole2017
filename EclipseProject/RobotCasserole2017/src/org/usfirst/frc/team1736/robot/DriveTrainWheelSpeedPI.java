package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.Calibration.Calibration;
import org.usfirst.frc.team1736.lib.CasserolePID.CasserolePID;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

public class DriveTrainWheelSpeedPI extends CasserolePID {
	
	SpeedController spdctrl;
	Encoder encoder;
	
	Calibration K_ff_cal;
	Calibration K_p_cal;
	Calibration K_i_cal;
	
	boolean enabled;

	public DriveTrainWheelSpeedPI(SpeedController spdctrl_in, Encoder encoder_in, Calibration K_ff_cal_in, Calibration K_p_cal_in, Calibration K_i_cal_in) {
		super(K_p_cal_in.get(), K_i_cal_in.get(), 0, K_ff_cal_in.get(),0,0);
		this.threadName = "Drivetrain Velocity PID";

		spdctrl = spdctrl_in;
		encoder = encoder_in;
		K_ff_cal = K_ff_cal_in;
		K_p_cal = K_p_cal_in;
		K_i_cal = K_i_cal_in;
		
		enabled = false;
		
		//Motor Controllers allow a fixed 1/-1 range
		this.setOutputRange(-1, 1);
	}
	
	/**
	 * Update whether this PI controller should be enabled or not
	 * PID threads are started/stopped when enable changes state.
	 * Integrators are also reset each time the controller is reset.
	 * @param enable_in true to run this controller, false to disable it.
	 */
	public void setEnabled(boolean enable_in){
		if(enable_in != enabled){
			//Only bother to do something if we're changing the enabled state.
			if(enable_in == true & enabled == false){
				//We used to be disabled, but now want to enable.
				this.resetIntegrators();
				this.start();
			} else if (enable_in == false & enabled == true){
				this.stop();
			}
			enabled = enable_in;
		}
		
	}
	
	/**
	 * Update the calibrations for the PI(f) controller
	 */
	public void updateCal(){
		
		//Note acknowledgments must be done elsewhere
		//since there will me many classes which use
		//the same calibrations here.
		if(K_ff_cal.isChanged()){
			this.setKf(K_ff_cal.get());
		}
		
		if(K_p_cal.isChanged()){
			this.setKp(K_p_cal.get());
		}
		
		if(K_i_cal.isChanged()){
			this.setKi(K_i_cal.get());
		}
	}

	@Override
	protected double returnPIDInput() {
		return encoder.getRate()*60.0; //Assume motor controller scaled in rev/sec, return RPM
	}

	@Override
	protected void usePIDOutput(double pidOutput) {
		if(enabled){
			spdctrl.set(pidOutput); //presumes pidOutput will be in -1/1 range
		}
	}

}
