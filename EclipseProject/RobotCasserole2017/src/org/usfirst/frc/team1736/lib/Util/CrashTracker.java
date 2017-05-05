package org.usfirst.frc.team1736.lib.Util;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;

public class CrashTracker {
	
	static boolean disabledRunOnce = false;
	static boolean autoRunOnce = false;
	static boolean teleopRunOnce = false;

	private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();

	public static void logRobotConstruction() {
        logMarker("robot construction");
    }
	
	public static void logRobotInit() {
		logMarker("robot init");
	}
	
	public static void logDisabledInit() {
		logMarker("disabled init");
		disabledRunOnce = false;
	}
	
	public static void logDisabledPeriodic() {
		if(!disabledRunOnce){
			logMarker("disabled periodic");
			disabledRunOnce = true;
		}
	}
	
	public static void logAutoInit() {
		logMarker("auto init");
	}
	
	public static void logAutoPeriodic() {
		if(!autoRunOnce){
			logMarker("auto periodic");
			autoRunOnce = true;
		}
	}
	
	public static void logTeleopInit() {
		logMarker("teleop init");
	}
	
	public static void logTeleopPeriodic() {
		if(!teleopRunOnce){
			logMarker("teleop periodic");
			teleopRunOnce = true;
		}
	}
		
	 public static void logThrowableCrash(Throwable throwable) {
	        logMarker("Exception", throwable);
	}
	
	private static void logMarker(String mark) {
	        logMarker(mark, null);
	}

	private static void logMarker(String mark, Throwable nullableException) {

	        try (PrintWriter writer = new PrintWriter(new FileWriter("/home/lvuser/crash_tracking.txt", true))) {
	            writer.print(RUN_INSTANCE_UUID.toString());
	            writer.print(", ");
	            writer.print(mark);
	            writer.print(", ");
	            writer.print(new Date().toString());

	            if (nullableException != null) {
	                writer.print(", ");
	                nullableException.printStackTrace(writer);
	            }

	            writer.println();
	        } catch (IOException e) {
	            e.printStackTrace();
	        }
	}
} 
