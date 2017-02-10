package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;

public class PneumaticsSupply {
	Compressor comp;
	
	private boolean enabled;
	
	private static AnalogInput psensor;
	
	public PneumaticsSupply(){
		comp = new Compressor();
		comp.setClosedLoopControl(true); //ensure we are running by default
		enabled = true;
		
		psensor = new AnalogInput(RobotConstants.AIR_PRESSURE_SENSOR_PORT);
	}
	
	/**
	 * 
	 * @return System unregulated pressure in psi
	 */
	public double getPress(){
		return ((psensor.getVoltage()/5.0)-0.1)*150.0/0.8;
	}
	
	/**\
	 * 
	 * @return Compressor current draw in amps
	 */
	public double getCompCurrent(){
		return comp.getCompressorCurrent();
	}
	
	/**
	 * 
	 * @param state pass True run the compressor up to top pressure, false to turn it off.
	 */
	public void setCompressorEnabled(boolean state){
		comp.setClosedLoopControl(state);
		if(state == false){
			comp.stop();
		}
		enabled = state;
	}
	
	/**
	 * 
	 * @return true if compressor is enabled to run, false if disabled.
	 */
	public boolean isEnabled(){
		return enabled;
	}

}
