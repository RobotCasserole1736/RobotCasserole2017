package org.usfirst.frc.team1736.robot;

/*
 *******************************************************************************************
 * Copyright (C) 2017 FRC Team 1736 Robot Casserole - www.robotcasserole.org
 *******************************************************************************************
 *
 * This software is released under the MIT Licence - see the license.txt
 *  file in the root of this repo.
 *
 * Non-legally-binding statement from Team 1736:
 *  Thank you for taking the time to read through our software! We hope you
 *   find it educational and informative! 
 *  Please feel free to snag our software for your own use in whatever project
 *   you have going on right now! We'd love to be able to help out! Shoot us 
 *   any questions you may have, all our contact info should be on our website
 *   (listed above).
 *  If you happen to end up using our software to make money, that is wonderful!
 *   Robot Casserole is always looking for more sponsors, so we'd be very appreciative
 *   if you would consider donating to our club to help further STEM education.
 */

import org.usfirst.frc.team1736.lib.AutoSequencer.AutoSequencer;
import org.usfirst.frc.team1736.lib.Calibration.Calibration;
import org.usfirst.frc.team1736.robot.auto.AutoEventCrossBaseLine;
import org.usfirst.frc.team1736.robot.auto.AutoEventMoveFromBlue;
import org.usfirst.frc.team1736.robot.auto.AutoEventMoveFromRed;
import org.usfirst.frc.team1736.robot.auto.AutoEventShoot;

public class Autonomous {
	Calibration autoMode;
	public Autonomous(){
		autoMode = new Calibration("Auto Mode",0,0,5);
	}
	
	public void executeAutonomus(){
		DriveTrain driveTrain = DriveTrain.getInstance();
		int mode = (int) Math.round(autoMode.get());
		switch(mode){
			case 1: //drive forward across base line
				AutoEventCrossBaseLine driveForward = new AutoEventCrossBaseLine(driveTrain.getFrontLeftCTRL(),driveTrain.getFrontRightCTRL(),driveTrain.getRearLeftCTRL(),driveTrain.getRearRightCTRL());
				AutoSequencer.addEvent(driveForward);
				break;
			case 2:
				AutoEventMoveFromBlue driveBlue = new AutoEventMoveFromBlue(driveTrain.getFrontLeftCTRL(),driveTrain.getFrontRightCTRL(),driveTrain.getRearLeftCTRL(),driveTrain.getRearRightCTRL());
				AutoSequencer.addEvent(driveBlue);
				AutoEventShoot shootNow = new AutoEventShoot();
				AutoSequencer.addEvent(shootNow);
				break;
			case 3:
				AutoEventMoveFromRed driveRed = new AutoEventMoveFromRed(driveTrain.getFrontLeftCTRL(),driveTrain.getFrontRightCTRL(),driveTrain.getRearLeftCTRL(),driveTrain.getRearRightCTRL());
				AutoSequencer.addEvent(driveRed);
				AutoEventShoot shootNow2 = new AutoEventShoot();
				AutoSequencer.addEvent(shootNow2);
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

