package org.usfirst.frc.team1736.robot;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.CameraServer;

public class CameraControl {
	
	public CameraControl(){
		
		//Start Vision Process Camera
		UsbCamera visionProcCam = new UsbCamera("VisionProcCam", 0);
		visionProcCam.setFPS(4); //this seems to not work for some reason???
		visionProcCam.setResolution(RobotConstants.VISION_X_PIXELS, RobotConstants.VISION_Y_PIXELS);
		visionProcCam.setExposureManual(5);
		visionProcCam.setWhiteBalanceManual(9000);
		MjpegServer server = new MjpegServer("VisionProcCamServer", 1181);
		server.setSource(visionProcCam);

	}


}
