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
import org.usfirst.frc.team1736.robot.auto.AutoEventBackAwayFromLiftNoCross;
import org.usfirst.frc.team1736.robot.auto.AutoEventBackAwayLeftFromLift;
import org.usfirst.frc.team1736.robot.auto.AutoEventBackAwayRightFromLift;
import org.usfirst.frc.team1736.robot.auto.AutoEventCrossBaseLine;
import org.usfirst.frc.team1736.robot.auto.AutoEventDriveToCenterLift;
import org.usfirst.frc.team1736.robot.auto.AutoEventMoveFromBlue;
import org.usfirst.frc.team1736.robot.auto.AutoEventMoveFromRed;
import org.usfirst.frc.team1736.robot.auto.AutoEventOpenGearMechanism;
import org.usfirst.frc.team1736.robot.auto.AutoEventShootNoVision;
import org.usfirst.frc.team1736.robot.auto.AutoEventShootWithVision;
import org.usfirst.frc.team1736.robot.auto.AutoEventTest;

public class Autonomous {
	Calibration autoMode;
	
	String autoModeName = "Not Initalized";
	
	int mode;
	
	public Autonomous(){
		autoMode = new Calibration("Auto Mode",0,0,10);
		mode = 100; //A number I think we will never use
	}
	
	public void updateAutoSelection(){

		if((int) Math.round(autoMode.get()) != mode){
			mode = (int) Math.round(autoMode.get());
			
			//The following must be aligned to the below selection
			switch(mode){
			case 1: //drive forward across base line
				autoModeName = "Cross Baseline";
				break;
			case 2: //put a gear on the center lift
				autoModeName = "Gear No X";
				break;
			case 3: //put a gear on the center lift
				autoModeName = "Gear X Left";
				break;
			case 4: //put a gear on the center lift
				autoModeName = "Gear X Right";
				break;
			case 5: //Shoot without vision alignment or motion
				autoModeName = "No Move Shoot";
				break;
			case 6: //Move off the blue wall, vision align and shoot
				autoModeName = "Vision Shoot Blue";
				break;
			case 7: //move off the red wall, vision align and shoot
				autoModeName = "Vision Shoot Red";
				break;
			case 8: //Shoot without vision alignment or motion
				autoModeName = "Test - DO NOT USE!";
				break;
			default: //Do nothing
				autoModeName = "Do Nothing";
				break;
			}
			System.out.println("[Auto] New mode selected: " + autoModeName);
		}
		

	}
	
	public void executeAutonomus(){
		
		System.out.println("[Auto] Initalizing " + autoModeName + " auton routine.");
		
		AutoSequencer.clearAllEvents();
		
		DriveTrain.getInstance().getFrontLeftCTRL().resetIntegrators();
		DriveTrain.getInstance().getFrontRightCTRL().resetIntegrators();
		DriveTrain.getInstance().getRearLeftCTRL().resetIntegrators();
		DriveTrain.getInstance().getRearRightCTRL().resetIntegrators();
		
		switch(mode){
			case 1: //drive forward across base line
				AutoEventCrossBaseLine driveForward = new AutoEventCrossBaseLine();
				AutoSequencer.addEvent(driveForward);
				break;
			case 2: //put a gear on the center lift, straight Back
				AutoEventDriveToCenterLift gearDeliverC = new AutoEventDriveToCenterLift();
				AutoSequencer.addEvent(gearDeliverC);
				AutoEventOpenGearMechanism openGearC = new AutoEventOpenGearMechanism();
				AutoSequencer.addEvent(openGearC);
				AutoEventBackAwayFromLiftNoCross backAwayC = new AutoEventBackAwayFromLiftNoCross();
				AutoSequencer.addEvent(backAwayC);
				break;
			case 3: //put a gear on the center lift, go to the left
				AutoEventDriveToCenterLift gearDeliverL = new AutoEventDriveToCenterLift();
				AutoSequencer.addEvent(gearDeliverL);
				AutoEventOpenGearMechanism openGearL = new AutoEventOpenGearMechanism();
				AutoSequencer.addEvent(openGearL);
				AutoEventBackAwayLeftFromLift backAwayL = new AutoEventBackAwayLeftFromLift();
				AutoSequencer.addEvent(backAwayL);
				break;
			case 4: //put a gear on the center lift, go to the right
				AutoEventDriveToCenterLift gearDeliverR = new AutoEventDriveToCenterLift();
				AutoSequencer.addEvent(gearDeliverR);
				AutoEventOpenGearMechanism openGearR = new AutoEventOpenGearMechanism();
				AutoSequencer.addEvent(openGearR);
				AutoEventBackAwayRightFromLift backAwayR = new AutoEventBackAwayRightFromLift();
				AutoSequencer.addEvent(backAwayR);
				break;

			case 5: //drive forward across base line
				AutoEventShootNoVision olShoot = new AutoEventShootNoVision();
				AutoSequencer.addEvent(olShoot);
				break;
				
			case 6:
				AutoEventMoveFromBlue driveBlue = new AutoEventMoveFromBlue();
				AutoSequencer.addEvent(driveBlue);
				AutoEventShootWithVision shootNow = new AutoEventShootWithVision();
				AutoSequencer.addEvent(shootNow);
				break;
			case 7:
				AutoEventMoveFromRed driveRed = new AutoEventMoveFromRed();
				AutoSequencer.addEvent(driveRed);
				AutoEventShootWithVision shootNow2 = new AutoEventShootWithVision();
				AutoSequencer.addEvent(shootNow2);
				break;
			case 8:
				AutoEventTest swagCross = new AutoEventTest();
				AutoSequencer.addEvent(swagCross);
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

