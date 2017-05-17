package org.usfirst.frc.team1736.lib.Util;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;

public class CrashTracker {
	
	static boolean consRunOnce = false;
	static boolean initRunOnce = false;
	static boolean disabledInitRunOnce = false;
	static boolean disabledPeriodicRunOnce = false;
	static boolean autoInitRunOnce = false;
	static boolean autoPeriodicRunOnce = false;
	static boolean teleopInitRunOnce = false;
	static boolean teleopPeriodicRunOnce = false;

	private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();

	public static void logRobotConstruction() {
        if(!consRunOnce){
        	logMarker("robot construction");
        	consRunOnce = true;
        }
    }
	
	public static void logRobotInit() {
		if(!initRunOnce){
			logMarker("robot init");
			initRunOnce = true;
		}
	}
	
	public static void logDisabledInit() {
		if(!disabledInitRunOnce){
			logMarker("disabled init");
			disabledInitRunOnce = true;
		}
	}
	
	public static void logDisabledPeriodic() {
		if(!disabledPeriodicRunOnce){
			logMarker("disabled periodic");
			disabledPeriodicRunOnce = true;
		}
	}
	
	public static void logAutoInit() {
		if(!autoInitRunOnce){
			logMarker("auto init");
			autoInitRunOnce= true;
		}
	}
	
	public static void logAutoPeriodic() {
		if(!autoPeriodicRunOnce){
			logMarker("auto periodic");
			autoPeriodicRunOnce = true;
		}
	}
	
	public static void logTeleopInit() {
		if(teleopInitRunOnce){
			logMarker("teleop init");
			teleopInitRunOnce = true;
		}
	}
	
	public static void logTeleopPeriodic() {
		if(!teleopPeriodicRunOnce){
			logMarker("teleop periodic");
			teleopPeriodicRunOnce = true;
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
