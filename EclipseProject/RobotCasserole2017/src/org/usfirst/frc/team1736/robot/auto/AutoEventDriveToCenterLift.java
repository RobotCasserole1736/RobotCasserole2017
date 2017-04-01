package org.usfirst.frc.team1736.robot.auto;

import org.usfirst.frc.team1736.lib.AutoSequencer.AutoEvent;
import org.usfirst.frc.team1736.lib.FalconPathPlanner.PathPlannerAutoEvent;
import org.usfirst.frc.team1736.robot.DriveTrain;
import org.usfirst.frc.team1736.robot.DriveTrainWheelSpeedPI;

import edu.wpi.first.wpilibj.Timer;

public class AutoEventDriveToCenterLift extends AutoEvent{
	
	private static final double TRAVEL_TIME_SEC = 4.5;
	private static final double MOTOR_SPEED_CMD_RPM = 500;
	
	private double startTime;
	private double endTime;
	
	public AutoEventDriveToCenterLift() {
		
	}
	
	public void userForceStop() {
		DriveTrain.getInstance().getFrontLeftCTRL().setSetpoint(0);
		DriveTrain.getInstance().getFrontRightCTRL().setSetpoint(0);
		DriveTrain.getInstance().getRearLeftCTRL().setSetpoint(0);
		DriveTrain.getInstance().getRearRightCTRL().setSetpoint(0);
		DriveTrain.getInstance().setAutonRequestsOpenLoop(false);
	}
	public boolean isTriggered() {
		//always run this event
		return true;
	}
	public boolean isDone() {
		if(Timer.getFPGATimestamp() > endTime){
			userForceStop();
			return true;
		} else{
			return false;
		}
		
	}

	@Override
	public void userUpdate() {
		DriveTrain.getInstance().getFrontLeftCTRL().setSetpoint(MOTOR_SPEED_CMD_RPM);
		DriveTrain.getInstance().getFrontRightCTRL().setSetpoint(MOTOR_SPEED_CMD_RPM);
		DriveTrain.getInstance().getRearLeftCTRL().setSetpoint(MOTOR_SPEED_CMD_RPM);
		DriveTrain.getInstance().getRearRightCTRL().setSetpoint(MOTOR_SPEED_CMD_RPM);
		
	}

	@Override
	public void userStart() {
		startTime = Timer.getFPGATimestamp();
		endTime = startTime + TRAVEL_TIME_SEC;
		DriveTrain.getInstance().setAutonRequestsOpenLoop(true);
	}
	
}
