package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.AutoSequencer.AutoSequencer;
import org.usfirst.frc.team1736.lib.Calibration.Calibration;
import org.usfirst.frc.team1736.robot.auto.AutoEventDriveForward;

public class Autonomus {
	Calibration autoMode;
	DriveTrain driveTrain;
	public Autonomus(DriveTrain driveTrain){
		autoMode = new Calibration("Auto Mode",0,0,5);
		this.driveTrain = driveTrain;
	}
	
	public void executeAutonomus(){
		int mode = (int) Math.round(autoMode.get());
		switch(mode){
			case 1: //drive forward across base line
				AutoEventDriveForward driveForward = new AutoEventDriveForward(driveTrain.getFrontLeftCTRL(),driveTrain.getFrontRightCTRL(),driveTrain.getRearLeftCTRL(),driveTrain.getRearrightCTRL());
				AutoSequencer.addEvent(driveForward);
				break;
				
		}
		AutoSequencer.start();
		
		
	}
	
	public void update(){
		AutoSequencer.update();
	}
	public void stop(){
		AutoSequencer.stop();
	}
}

