package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.CasserolePID.CasserolePID;;

public class VisionAlignDistPID extends CasserolePID {

	protected VisionAlignDistPID(double Kp_in, double Ki_in, double Kd_in) {
		super(Kp_in, Ki_in, Kd_in);
		// TODO Auto-generated constructor stub
	}
	public void setDist(double dist){
		if(dist == 0.0){
			resetIntegrators();
		}
		
		setSetpoint(dist);
	}
	
	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		return RobotState.visionEstTargetDist_ft;
	}

	@Override
	protected void usePIDOutput(double pidOutput) {
		// TODO Auto-generated method stub
		RobotState.visionDtFwdRevCmd = pidOutput;	
	}

}
