package org.usfirst.frc.team1736.robot;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.CameraServer;

public class CameraControl {
	CvSource outputImage;
	CvSink fromCam;
	Mat img_tmp;
	
	public CameraControl(){
		
		//Start camera
		/*
		UsbCamera visionProcCam = new UsbCamera("VisionProcCam", 0);
		visionProcCam.setFPS(1);
		visionProcCam.setResolution(RobotConstants.VISION_X_PIXELS, RobotConstants.VISION_Y_PIXELS);
		visionProcCam.setExposureManual(0);
		visionProcCam.setWhiteBalanceManual(9000);
		MjpegServer server = new MjpegServer("VisionProcCamServer", 1181);
		server.setSource(visionProcCam);
		
		*/
		
		img_tmp = new Mat();
		
		UsbCamera visionProcCam = new UsbCamera("VisionProcCam", 0);
		visionProcCam.setFPS(4);
		visionProcCam.setResolution(RobotConstants.VISION_X_PIXELS, RobotConstants.VISION_Y_PIXELS);
		visionProcCam.setExposureManual(0);
		visionProcCam.setWhiteBalanceManual(9000);
		
		fromCam = new CvSink("CameraInput");
		fromCam.setSource(visionProcCam);

		outputImage = new CvSource("VisionProcCamDownsampledOutput", VideoMode.PixelFormat.kMJPEG, RobotConstants.VISION_X_PIXELS, RobotConstants.VISION_Y_PIXELS, 4);
		
		MjpegServer server = new MjpegServer("VisionProcCamServer", 1181);
		server.setSource(outputImage);

	}
	
	public void sampleVisionProcImg(){
		//Sample Frame
		fromCam.grabFrame(img_tmp);
		outputImage.putFrame(img_tmp);
	}

}
