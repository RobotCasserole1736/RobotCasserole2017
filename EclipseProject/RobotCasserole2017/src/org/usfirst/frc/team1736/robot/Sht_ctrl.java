package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.Calibration.Calibration;

public class Sht_ctrl {

	 Calibration wheel_Set_Point_rpm;
	 public Sht_ctrl(){ 
		 wheel_Set_Point_rpm = new Calibration("Shooter Wheel Setpoint RPM", 2700, 0, 5000);
		 RobotState.hopperFeedCmd = false;
		 RobotState.shooterDesiredVelocity_rpm= 0;
	 }

		 
	 public void update(){
		  if(Shooter_States.NO_Shoot==RobotState.opShotCTRL){
			  //Operator requests everything turned off.
			  RobotState.hopperFeedCmd=false;
			  RobotState.shooterDesiredVelocity_rpm=0; 
		  }
		  else if(Shooter_States.PREP_TO_SHOOT==RobotState.opShotCTRL){
			  //Operator wants to prepare to shoot. Today, this means spooling up the shooter wheel.
			  RobotState.hopperFeedCmd=false;
			  RobotState.shooterDesiredVelocity_rpm=wheel_Set_Point_rpm.get();
		  }
		  else if((Shooter_States.SHOOT==RobotState.opShotCTRL) & RobotState.shooterVelocityOk){
			  //Operator wants to take the shot, and the shooter RPM is up to speed
			  if(RobotState.visionAlignmentDesiried){
				  //Driver has robot under automatic (vision-assist) alignment. 
				  if(RobotState.visionAlignmentPossible & RobotState.isVisionAlignmentOnTarget()){
					  //Vision alignment reports we are on target. Take the shot.
					  RobotState.hopperFeedCmd=true;
					  RobotState.shooterDesiredVelocity_rpm=wheel_Set_Point_rpm.get();
				  } else {
					  //Inhibit shot until vision alignment is achieved
					  RobotState.hopperFeedCmd=false;
					  RobotState.shooterDesiredVelocity_rpm=wheel_Set_Point_rpm.get();
				  }
			  } else {
				  //Shooter is under manual alignment, just take the shot if RPM is ok
				  RobotState.hopperFeedCmd=true;
				  RobotState.shooterDesiredVelocity_rpm=wheel_Set_Point_rpm.get();
			  }
		  }
		  else{ //Shot desired but wheel RPM is not OK
			  //Just spool the wheel back up. Hopefully we get it fast enough to take a shot soon.
			  RobotState.hopperFeedCmd=false;
			  RobotState.shooterDesiredVelocity_rpm=wheel_Set_Point_rpm.get();  
		  }
		 }
	}
 
		 
