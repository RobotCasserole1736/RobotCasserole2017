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
import org.usfirst.frc.team1736.robot.auto.AutoEventBackAwayFromLift;
import org.usfirst.frc.team1736.robot.auto.AutoEventCrossBaseLine;
import org.usfirst.frc.team1736.robot.auto.AutoEventDriveToCenterLift;
import org.usfirst.frc.team1736.robot.auto.AutoEventMoveFromBlue;
import org.usfirst.frc.team1736.robot.auto.AutoEventMoveFromRed;
import org.usfirst.frc.team1736.robot.auto.AutoEventOpenGearMechanism;
import org.usfirst.frc.team1736.robot.auto.AutoEventShootNoVision;
import org.usfirst.frc.team1736.robot.auto.AutoEventShootWithVision;

public class Autonomous {
	Calibration autoMode;
	
	String autoModeName = "Not Initalized";
	
	int mode;
	
	public Autonomous(){
		autoMode = new Calibration("Auto Mode",0,0,10);
	}
	
	public void updateAutoSelection(){
		mode = (int) Math.round(autoMode.get());
		
		//The following must be aligned to the below selection
		switch(mode){
		case 1: //drive forward across base line
			autoModeName = "Cross Baseline";
			break;
		case 2:
			autoModeName = "Vision Shoot Blue";
			break;
		case 3:
			autoModeName = "Vision Shoot Red";
			break;
		case 4: //put a gear on the center lift
			autoModeName = "Gear";
			break;
		case 5:
			autoModeName = "No Move Shoot";
		default: //Do nothing
			autoModeName = "Do Nothing";
			break;
		}
	}
	
	public void executeAutonomus(){
		
		switch(mode){
			case 1: //drive forward across base line
				AutoEventCrossBaseLine driveForward = new AutoEventCrossBaseLine();
				AutoSequencer.addEvent(driveForward);
				break;
			case 2:
				AutoEventMoveFromBlue driveBlue = new AutoEventMoveFromBlue();
				AutoSequencer.addEvent(driveBlue);
				AutoEventShootWithVision shootNow = new AutoEventShootWithVision();
				AutoSequencer.addEvent(shootNow);
				break;
			case 3:
				AutoEventMoveFromRed driveRed = new AutoEventMoveFromRed();
				AutoSequencer.addEvent(driveRed);
				AutoEventShootWithVision shootNow2 = new AutoEventShootWithVision();
				AutoSequencer.addEvent(shootNow2);
				break;
			case 4: //put a gear on the center lift
				AutoEventDriveToCenterLift gearDeliver = new AutoEventDriveToCenterLift();
				AutoSequencer.addEvent(gearDeliver);
				AutoEventOpenGearMechanism openGear = new AutoEventOpenGearMechanism();
				AutoSequencer.addEvent(openGear);
				AutoEventBackAwayFromLift backAway = new AutoEventBackAwayFromLift();
				AutoSequencer.addEvent(backAway);
				break;
			case 5: //drive forward across base line
				AutoEventShootNoVision olShoot = new AutoEventShootNoVision();
				AutoSequencer.addEvent(olShoot);
				break;
			default: //Do nothing
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

