package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.Calibration.Calibration;

public class Sht_ctrl {

	 Calibration wheel_Set_Point_rpm;
	 public Sht_ctrl(){ 
		 wheel_Set_Point_rpm = new Calibration("Shooter Wheel Setpoint (RPM)", 2700, 0, 5000);
		 RobotState.hopperFeedCmd = false;
		 RobotState.shooterDesiredVelocity_rpm= 0;
	 }

		 
	 public void update(){
		  if(Shooter_States.NO_Shoot==RobotState.opShotCTRL){
			  RobotState.hopperFeedCmd=false;
			  RobotState.shooterDesiredVelocity_rpm=0; 
		  }
		  else if(Shooter_States.PREP_TO_SHOOT==RobotState.opShotCTRL){
			  RobotState.hopperFeedCmd=false;
			  RobotState.shooterDesiredVelocity_rpm=wheel_Set_Point_rpm.get();
		  }
		  else if((Shooter_States.SHOOT==RobotState.opShotCTRL) & RobotState.shooterVelocityOk){
			  RobotState.hopperFeedCmd=true;
			  RobotState.shooterDesiredVelocity_rpm=wheel_Set_Point_rpm.get();
		  }
		  else{
			  RobotState.hopperFeedCmd=false;
			  RobotState.shooterDesiredVelocity_rpm=wheel_Set_Point_rpm.get();  
		  }
		 }
	}
 
		 
