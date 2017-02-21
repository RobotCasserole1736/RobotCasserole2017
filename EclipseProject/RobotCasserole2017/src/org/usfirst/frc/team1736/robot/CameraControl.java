package org.usfirst.frc.team1736.robot;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.util.HashMap;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;

public class CameraControl {
	
	private HashMap<String, UsbCamera> driverCameras = new HashMap<String, UsbCamera>();
	private MjpegServer driverStream = null;
	
	public CameraControl(){
		UsbCamera visionProcCam;
		UsbCamera logitech1;
		UsbCamera logitech2;
		
		//Start Vision Process Camera
		visionProcCam = new UsbCamera("Lifecam", getCameraDeviceNumber(RobotConstants.LIFECAM_USB_DEVICE_ID_SERIAL));
		logitech1 = new UsbCamera("Logitech_1", getCameraDeviceNumber(RobotConstants.LOGITECH_USB_DEVICE_ID_SERIAL_1));
		logitech2 = new UsbCamera("Logitech_2", getCameraDeviceNumber(RobotConstants.LOGITECH_USB_DEVICE_ID_SERIAL_2));
		driverCameras.put(RobotConstants.LIFECAM_USB_DEVICE_ID_SERIAL, visionProcCam);
		driverCameras.put(RobotConstants.LOGITECH_USB_DEVICE_ID_SERIAL_1, logitech1);
		driverCameras.put(RobotConstants.LOGITECH_USB_DEVICE_ID_SERIAL_2, logitech2);
		
		
		visionProcCam.setFPS(15); //this seems to not work for some reason???
		visionProcCam.setResolution(RobotConstants.VISION_X_PIXELS, RobotConstants.VISION_Y_PIXELS);
		visionProcCam.setExposureManual(5);
		visionProcCam.setWhiteBalanceManual(9000);
		MjpegServer visionCamServerHighRes = new MjpegServer("VisionProcCamServer", 1181);
		visionCamServerHighRes.setSource(visionProcCam);
		
		System.out.println(visionProcCam.getDescription());
		
		//Start Driver Camera
		logitech1.setResolution(320, 240);
		logitech1.setFPS(15);
		logitech2.setResolution(320, 240);
		logitech2.setFPS(15);
		driverStream = new MjpegServer("DriverCamServer", 1182);
		driverStream.setSource(logitech1);
		
		System.out.println(logitech1.getDescription());
		System.out.println(logitech2.getDescription());


	}
	
	public static int getCameraDeviceNumber(String serialId){
		for(int i = 0; i < 4; i++)
		{
			try {
				Process p = Runtime.getRuntime().exec("udevadm info --query=all --name=/dev/video" + i + " | grep ID_SERIAL=");
				p.waitFor();
				BufferedReader reader = new BufferedReader(new InputStreamReader(p.getInputStream()));
				String line = "";
				while((line = reader.readLine()) != null){
					if(line.contains(serialId))
						return i;
					if(line.contains("device node not found"))
						return -1;
				}
			} catch (Exception e) {
				e.printStackTrace();
				return -1;
			}
		}
		return -1;
	}

	public void setDriverCamera(String serialId){
		if(driverStream.getSource().equals(driverCameras.get(serialId)))
			return;
		driverStream.setSource(driverCameras.get(serialId));
	}

}
