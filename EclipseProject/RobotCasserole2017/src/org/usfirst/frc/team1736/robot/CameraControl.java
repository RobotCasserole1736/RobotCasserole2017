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
		UsbCamera visionProcCam;
		UsbCamera DriverCam;
		
		//Start Vision Process Camera
		UsbCamera cam0 = new UsbCamera("Camera0", 0);
		UsbCamera cam1 = new UsbCamera("Camera1", 1);
		
		
		
		if(cam0.getDescription().contains("Microsoft")){
			//Cam 0 is for vision processing
			visionProcCam = cam0;
			DriverCam = cam1;
		} else {
			//inverted
			visionProcCam = cam1;
			DriverCam = cam0;
		}
		
		
		visionProcCam.setFPS(4); //this seems to not work for some reason???
		visionProcCam.setResolution(RobotConstants.VISION_X_PIXELS, RobotConstants.VISION_Y_PIXELS);
		visionProcCam.setExposureManual(5);
		visionProcCam.setWhiteBalanceManual(9000);
		MjpegServer visionCamServerHighRes = new MjpegServer("VisionProcCamServer", 1181);
		visionCamServerHighRes.setSource(visionProcCam);
		
		System.out.println(visionProcCam.getDescription());
		
		//Start Driver Camera
		
		DriverCam.setResolution(640, 480);
		MjpegServer driverCamServer = new MjpegServer("VisionProcCamServer", 1182);
		driverCamServer.setSource(DriverCam);
		
		System.out.println(DriverCam.getDescription());


	}


}
