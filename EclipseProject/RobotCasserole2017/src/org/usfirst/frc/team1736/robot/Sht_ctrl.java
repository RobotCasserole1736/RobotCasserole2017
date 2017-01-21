package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.Calibration.Calibration;

public class Sht_ctrl {
	 ShooterWheelCTRL shooterControl;
	 HopperControl hopControl;
	 public Sht_ctrl(){
	 Calibration wheel_Set_Point_rpm;	 
			 
		 
	 public void update(){
		 if((Shooter_States.SHOOT== RobotState.opShotCTRL) & RobotState.shooterVelocityOk){
		 RobotState.hopperFeedCmd=true;
		 }
		 else{
			 Shooter_States.NO_Shoot ;
		{ 
			if(){
				
			}
		}
	 }
