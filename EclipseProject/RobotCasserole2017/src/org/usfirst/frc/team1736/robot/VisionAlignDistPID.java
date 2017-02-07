package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.CasserolePID.CasserolePID;;

public class VisionAlignDistPID extends CasserolePID {
	
	private double outputCmd = 0;

	protected VisionAlignDistPID(double Kp_in, double Ki_in, double Kd_in) {
		super(Kp_in, Ki_in, Kd_in);
		this.threadName = "Vision Distance Alignment PID";
	
	}
	
	public void setDist(double dist){
		setSetpoint(dist);
	}
	
	@Override
	protected double returnPIDInput() {
		return RobotPoseCalculator.getInstance().getFwdRevDistFt();
	}

	@Override
	protected void usePIDOutput(double pidOutput) {
		outputCmd = pidOutput;	
	}
	
	@Override
	public void stop() {
		super.stop();
		outputCmd = 0;
	}
	
	public double getOutputCommand()
	{
		return outputCmd;
	}

}
