package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.CasserolePID.CasserolePID;

public class VisionAlignAnglePID extends CasserolePID {

	VisionAlignAnglePID(double Kp_in, double Ki_in, double Kd_in) {
		super(Kp_in, Ki_in, Kd_in);
		this.threadName = "Vision Angle Alignment PID";
		
	}
	
	public void setAngle(double angle){		
		setSetpoint(angle);
	}
	
	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		return Gyro.getInstance().getAngle();
	}

	@Override
	protected void usePIDOutput(double pidOutput) {
		//Limit to half range to reduce overshoot
		RobotState.visionDtRotateCmd = pidOutput;
	}

}
