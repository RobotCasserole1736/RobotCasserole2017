package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.VictorSP;

public class RateLimitedVictorSP extends VictorSP {

	double prevSetpoint = 0;
	
	public final double PER_LOOP_SETPOINT_RATE_LIMIT = 0.2;
	
	public RateLimitedVictorSP(int channel) {
		super(channel);

	}
	
	@Override
	public void set(double val){
		double delta = val - prevSetpoint;
		
		if(delta > PER_LOOP_SETPOINT_RATE_LIMIT){
			super.set(prevSetpoint + PER_LOOP_SETPOINT_RATE_LIMIT);
		} else if(delta < -PER_LOOP_SETPOINT_RATE_LIMIT){
			super.set(prevSetpoint - PER_LOOP_SETPOINT_RATE_LIMIT);
		} else {
			super.set(val);
		}
		
		prevSetpoint = val;
		
	}

}
