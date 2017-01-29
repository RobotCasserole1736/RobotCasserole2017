package org.usfirst.frc.team1736.robot;

import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JFrame;

import org.usfirst.frc.team1736.lib.LEDs.CasseroleLEDInterface;
import org.usfirst.frc.team1736.lib.LEDs.DesktopTestLEDs;

public class LEDSequencer {

	static DesktopTestLEDs ledstrip;
    Timer timerThread;
	
    public final int NUM_LEDS_TOTAL = 40;
    
    int loop_counter;
    
	public LEDSequencer(){
		ledstrip = new DesktopTestLEDs(NUM_LEDS_TOTAL);
		
		loop_counter = 0;
		
		//Start LED animation thread in background.
        timerThread = new java.util.Timer("LED Sequencer Update");
        timerThread.schedule(new LEDBackgroundUpdateTask(this), (long) (CasseroleLEDInterface.m_update_period_ms), (long) (CasseroleLEDInterface.m_update_period_ms));
	}
	
	public void update(){
		
		smoothStripSweep();
		
		
		loop_counter++;
	}
	
	private void smoothStripSweep(){
		ledstrip.clearColorBuffer();
		
		for(int led_idx = 0; led_idx < NUM_LEDS_TOTAL; led_idx++ ){
			double not_red_comp = Math.min(1, Math.max(0, 1.5*(0.5+Math.sin((led_idx/2.0 + loop_counter/5.0)))));
			ledstrip.setLEDColor(led_idx, 1, not_red_comp, not_red_comp);

		}
	}
	
	
    // Java multithreading magic. Do not touch.
    // Touching will incour the wrath of Cthulhu, god of java and LED Strips.
    // May the oceans of 1's and 0's rise to praise him.
    private class LEDBackgroundUpdateTask extends TimerTask {
        private LEDSequencer m_sequencer;


        public LEDBackgroundUpdateTask(LEDSequencer sequencer) {
            if (sequencer == null) {
                throw new NullPointerException("Given Desktop LEDs Controller Class was null");
            }
            
            
            m_sequencer = sequencer;
        }


        @Override
        public void run() {
        	m_sequencer.update();
        }
    }

    //Main method for testing locally on a PC
	public static void main(String[] args){
		LEDSequencer seq = new LEDSequencer();
		
	    JFrame frame = new JFrame("LED Test");
	    frame.setSize(750, 200);
	    frame.setVisible(true);
	    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	    frame.add(ledstrip);
	}
}
