package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.Calibration.Calibration;

public class ShotControl {
	private static ShotControl shotControl = null;
	Calibration wheel_Set_Point_rpm;
	private boolean hopperFeedCmd;
	private ShooterWheelCtrl wheelCtrl;
	private ShooterStates desiredShooterState = ShooterStates.NO_Shoot;
	
	public enum ShooterStates 
	{
		NO_Shoot,PREP_TO_SHOOT,SHOOT;	
	}
	
	public static synchronized ShotControl getInstance()
	{
		if(shotControl == null)
			shotControl = new ShotControl();
		return shotControl;
	}

	 private ShotControl(){ 
		 wheel_Set_Point_rpm = new Calibration("Shooter Wheel Setpoint RPM", 2700, 0, 5000);
		 hopperFeedCmd = false;
		 wheelCtrl = ShooterWheelCtrl.getInstance();
		 wheelCtrl.setShooterDesiredRPM(0);
	 }

		 
	 public void update(){
		  if(ShooterStates.NO_Shoot == desiredShooterState){
			  //Operator requests everything turned off.
			  hopperFeedCmd=false;
			  wheelCtrl.setShooterDesiredRPM(0);
		  }
		  else if(ShooterStates.PREP_TO_SHOOT == desiredShooterState){
			  //Operator wants to prepare to shoot. Today, this means spooling up the shooter wheel.
			  hopperFeedCmd=false;
			  wheelCtrl.setShooterDesiredRPM(wheel_Set_Point_rpm.get());
		  }
		  else if((ShooterStates.SHOOT == desiredShooterState) & wheelCtrl.getShooterVelocityOK()){
			  //Operator wants to take the shot, and the shooter RPM is up to speed
			  if(VisionAlignment.getInstance().getVisionAlignmentDesired()){
				  //Driver has robot under automatic (vision-assist) alignment. 
				  if(VisionAlignment.getInstance().getVisionAlignmentPossible() & VisionAlignment.getInstance().getVisionAlignmentOnTarget()){
					  //Vision alignment reports we are on target. Take the shot.
					  hopperFeedCmd=true;
					  wheelCtrl.setShooterDesiredRPM(wheel_Set_Point_rpm.get());
				  } else {
					  //Inhibit shot until vision alignment is achieved
					  hopperFeedCmd=false;
					  wheelCtrl.setShooterDesiredRPM(wheel_Set_Point_rpm.get());
				  }
			  } else {
				  //Shooter is under manual alignment, just take the shot if RPM is ok
				  hopperFeedCmd=true;
				  wheelCtrl.setShooterDesiredRPM(wheel_Set_Point_rpm.get());
			  }
		  }
		  else{ //Shot desired but wheel RPM is not OK
			  //Just spool the wheel back up. Hopefully we get it fast enough to take a shot soon.
			  hopperFeedCmd=false;
			  wheelCtrl.setShooterDesiredRPM(wheel_Set_Point_rpm.get());
		  }
	 }
	 
	 public boolean getHopperFeedCmd()
	 {
		 return hopperFeedCmd;
	 }
	 
	 public ShooterStates getDesiredShooterState()
	 {
		 return desiredShooterState;
	 }
	 
	 public void setDesiredShooterState(ShooterStates state)
	 {
		 desiredShooterState = state;
	 }
}
 
		 
