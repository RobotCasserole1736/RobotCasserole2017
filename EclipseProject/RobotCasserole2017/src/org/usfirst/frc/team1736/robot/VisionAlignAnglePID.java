package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.CasserolePID.CasserolePID;

public class VisionAlignAnglePID extends CasserolePID {

	VisionAlignAnglePID(double Kp_in, double Ki_in, double Kd_in) {
		super(Kp_in, Ki_in, Kd_in);
		
	}
	
	public void setAngle(double angle){		
		setSetpoint(angle);
	}
	
	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		return RobotState.robotPoseAngle_deg;
	}

	@Override
	protected void usePIDOutput(double pidOutput) {
		// Negate to get proper turn based on error sign.
		//Limit to half range to reduce overshoot
		RobotState.visionDtRotateCmd = Math.min(0.5, Math.max(-0.5, pidOutput));
	}

}
