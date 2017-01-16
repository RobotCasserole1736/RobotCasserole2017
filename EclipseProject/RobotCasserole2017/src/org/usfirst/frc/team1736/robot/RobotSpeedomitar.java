package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.WebServer.CasseroleDriverView;

public class RobotSpeedomitar {
	public RobotSpeedomitar(){
		CasseroleDriverView.newDial("RobotSpeed ftperse", 0, 25, 5, 0, 20);
	}
	public void update(){
		double WheelSpeedOno;
			WheelSpeedOno =RobotState.frontLeftWheelVelocity_rpm * 2.0 * Math.PI / 60; 
		double WheelSpeedDos;
			WheelSpeedDos =RobotState.frontRightWheelVelocity_rpm * 2.0 * Math.PI / 60;
		double WheelSpeedTres;
			WheelSpeedTres =RobotState.rearLeftWheelVelocity_rpm * 2.0 * Math.PI / 60;
		double WheelSpeedCuatro;
			WheelSpeedCuatro =RobotState.rearRightWheelVelocity_rpm * 2.0 * Math.PI / 60;
		double Vx;
			Vx = (WheelSpeedOno + WheelSpeedDos + WheelSpeedTres + WheelSpeedCuatro) * 0.17 / 4;
		double Vy;
			Vy  = (-WheelSpeedOno + WheelSpeedDos - WheelSpeedTres + WheelSpeedCuatro) * 0.17 / 4;
		double netSpeed;
			netSpeed = Math.sqrt(Vx*Vx+Vy*Vy);
		CasseroleDriverView.setDialValue("RobotSpeed ftperse", netSpeed);	
	}
}
