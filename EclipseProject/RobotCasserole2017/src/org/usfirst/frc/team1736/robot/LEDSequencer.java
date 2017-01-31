package org.usfirst.frc.team1736.robot;

import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JFrame;

import org.usfirst.frc.team1736.lib.LEDs.CasseroleLEDInterface;
import org.usfirst.frc.team1736.lib.LEDs.DesktopTestLEDs;
import org.usfirst.frc.team1736.lib.LEDs.DotStarsLEDStrip;

public class LEDSequencer {

	//static DesktopTestLEDs ledstrip;
	DotStarsLEDStrip ledstrip;
    Timer timerThread;
	
    public final int NUM_LEDS_TOTAL = 40;
    
    int loop_counter;
    
	public LEDSequencer(){
		//ledstrip = new DesktopTestLEDs(NUM_LEDS_TOTAL);
		ledstrip = new DotStarsLEDStrip(NUM_LEDS_TOTAL);
		
		loop_counter = 0;
		
		//Start LED animation thread in background.
        timerThread = new java.util.Timer("LED Sequencer Update");
        timerThread.schedule(new LEDBackgroundUpdateTask(this), (long) (CasseroleLEDInterface.m_update_period_ms), (long) (CasseroleLEDInterface.m_update_period_ms));
	}
	
	public void update(){
		
		smoothStripSweep();
		//smoothRainbowCycle();
		//smoothRedWhiteCycle();
		//sparkleWhite();
		//sparkleRedWhite();
		//sparkleRainbow();
		//cylon();
		//cometRed();
		//cometRainbow();
		//bounce();
		
		
		loop_counter++;
	}
	
	//shift through all colors
	@SuppressWarnings("unused")
	private void smoothRainbowCycle(){
		final double period = 200; //Bigger makes it change color slower
		
		for(int led_idx = 0; led_idx < NUM_LEDS_TOTAL; led_idx++ ){
			double hue   = Math.abs((loop_counter)%period - period/2)/(period/2);
			ledstrip.setLEDColorHSL(led_idx, hue, 1, 0.5);
		}
	}
	
	//shift through Casserole red/white
	@SuppressWarnings("unused")
	private void smoothRedWhiteCycle(){
		final double period = 200; //Bigger makes it change color slower
		
		for(int led_idx = 0; led_idx < NUM_LEDS_TOTAL; led_idx++ ){
			double lightness   = Math.abs((loop_counter)%period - period/2)/(period/2);
			ledstrip.setLEDColorHSL(led_idx, 0, 1, lightness);
		}
	}
	
	
	//Similar to 2016 - does a smooth sweep of casserole red/white stripes
	@SuppressWarnings("unused")
	private void smoothStripSweep(){
		
		final double width = 2.0; //bigger means wider color strips
		final double period = 5.0; //bigger means slower cycle
		final double edgeSharpness = 1.0; //bigger means less blurred edges between stripe colors
		
		for(int led_idx = 0; led_idx < NUM_LEDS_TOTAL; led_idx++ ){
			double not_red_comp = Math.min(1, Math.max(0, (0.5+edgeSharpness*Math.sin((led_idx/width + loop_counter/period)))));
			ledstrip.setLEDColor(led_idx, 1, not_red_comp, not_red_comp);

		}
	}
	
	//shiny blips of white
	@SuppressWarnings("unused")
	private void sparkleWhite(){
		final double density = 0.05; //0 means never on, 1 means always on
		
		for(int led_idx = 0; led_idx < NUM_LEDS_TOTAL; led_idx++ ){
			double rand = Math.random();
			if(rand < density){
				ledstrip.setLEDColor(led_idx, 1, 1, 1);
				led_idx++;//make sure we separate blips by at least one space
			} else {
				ledstrip.setLEDColor(led_idx, 0, 0, 0);
			}
		}
	}
	
	//shiny blips of white and red
	@SuppressWarnings("unused")
	private void sparkleRedWhite(){
		final double density = 0.1; //0 means never on, 1 means always on
		
		for(int led_idx = 0; led_idx < NUM_LEDS_TOTAL; led_idx++ ){
			double rand = Math.random();
			if(rand < density){
				if(rand < density/2){
					ledstrip.setLEDColor(led_idx, 1, 1, 1);
				} else {
					ledstrip.setLEDColor(led_idx, 1, 0, 0);
				}
				led_idx++;//make sure we separate blips by at least one space
			} else {
				ledstrip.setLEDColor(led_idx, 0, 0, 0);
			}
		}
	}
	
	//shiny blips of all colors
	@SuppressWarnings("unused")
	private void sparkleRainbow(){
		final double density = 0.2; //0 means never on, 1 means always on
		
		for(int led_idx = 0; led_idx < NUM_LEDS_TOTAL; led_idx++ ){
			double rand = Math.random();
			if(rand < density){
				ledstrip.setLEDColorHSL(led_idx, rand/density, 1, 0.5);
				led_idx++;//make sure we separate blips by at least one space
			} else {
				ledstrip.setLEDColor(led_idx, 0, 0, 0);
			}
		}
	}
	
	//Evil robots of battelstar galactica fame
	@SuppressWarnings("unused")
	private void cylon(){
		
		final double width = 2.0; //bigger means wider on-width
		final int period = 50; //bigger means slower cycle
		
		double midpoint = (double)(Math.abs(((loop_counter)%period) - period/2))/((double)(period/2.0))*(NUM_LEDS_TOTAL/2.0); 
		
		for(int led_idx = 0; led_idx < NUM_LEDS_TOTAL/2; led_idx++ ){
			double red_val = Math.max(0, Math.min(1.0, 1-(0.15*Math.pow((midpoint-led_idx),2))));
			ledstrip.setLEDColor(led_idx, red_val, 0, 0);
			ledstrip.setLEDColor(led_idx+NUM_LEDS_TOTAL/2, red_val, 0, 0);

		}
	}
	
	//red comet shoots across the sky
	@SuppressWarnings("unused")
	private void cometRed(){
		
		final double width = 12.0; //bigger means wider on-width
		final int period = 30; //bigger means slower cycle
		
		double red_val;
		
		double endpoint = (double)(((loop_counter)%period)/((double)period))*((NUM_LEDS_TOTAL+width*10)/2.0); 
		
		for(int led_idx = 0; led_idx < NUM_LEDS_TOTAL/2; led_idx++ ){
			if(led_idx <= endpoint+2 ){
				red_val = Math.max(0, Math.min(1.0, (1-(endpoint-led_idx)/width)));
			} else {
				red_val = 0;
			}
			ledstrip.setLEDColor(led_idx, red_val, 0, 0);
			ledstrip.setLEDColor(led_idx+NUM_LEDS_TOTAL/2, red_val, 0, 0);

		}
	}
		
	//Colorful comets on a dark red background
	@SuppressWarnings("unused")
	private void cometRainbow(){
		
		final double width = 15.0; //bigger means wider on-width
		final int period = 30; //bigger means slower cycle
		
		double val;
		
		double endpoint = (double)(((loop_counter)%period)/((double)period))*((NUM_LEDS_TOTAL+width*10)/2.0); 
		
		for(int led_idx = 0; led_idx < NUM_LEDS_TOTAL/2; led_idx++ ){
			if(led_idx <= endpoint+2 ){
				val = Math.max(0, Math.min(1.0, (1-(endpoint-led_idx)/width)));
			} else {
				val = 0;
			}
			ledstrip.setLEDColorHSL(led_idx, val, 1, (val*0.5)+0.25);
			ledstrip.setLEDColorHSL(led_idx+NUM_LEDS_TOTAL/2, val, 1, (val*0.5)+0.25);

		}
	}
	
	//Globals needed for bouncing
	double pos1 = 5;
	double pos2 = 8;
	double vel1 = 0.4;
	double vel2 = -0.8;
	
	//Bouncing Balls
	@SuppressWarnings("unused")
	private void bounce(){

		//Super simple mostly-elastic colisison model
		if(pos1 <= 0 | pos1 >= NUM_LEDS_TOTAL/2){
			vel1 = -vel1*0.95;
		}
		if(pos2 <= 0 | pos2 >= NUM_LEDS_TOTAL/2){
			vel2 = -vel2*0.95;
		}
		if(Math.abs(pos1-pos2) < Math.max(Math.abs(vel1), Math.abs(vel2))){ //collision
			double tmp = vel1*0.95;
			vel1 = vel2*0.95;
			vel2 = tmp;
		}
		if(Math.abs(vel1) < 0.01 & Math.abs(vel2) < 0.01){ //reset
			vel1 = Math.random();
			vel2 = -Math.random();
			pos1 = 5;
			pos2 = 15;
		}
		
		if(pos1 > 0){
			//vel1 = vel1 - 0.05;
		}
		if(pos2 > 0){
			//vel2 = vel2 - 0.05;
		}
		
		pos1 += vel1;
		pos2 += vel2;
		
		
		for(int led_idx = 0; led_idx < NUM_LEDS_TOTAL/2; led_idx++ ){
			double val1 = Math.max(0, Math.min(1.0, 1-(0.75*Math.pow((pos1-led_idx),2))));
			double val2 = Math.max(0, Math.min(1.0, 1-(0.75*Math.pow((pos2-led_idx),2))));
			ledstrip.setLEDColor(led_idx, val1, 0, val2);
			ledstrip.setLEDColor(led_idx+NUM_LEDS_TOTAL/2, val1, 0, val2);

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
	@SuppressWarnings("unused")
	public static void main(String[] args){
		LEDSequencer seq = new LEDSequencer();
		
	    JFrame frame = new JFrame("LED Test");
	    frame.setSize(750, 200);
	    frame.setVisible(true);
	    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	    //frame.add(ledstrip); //uncomment this to do a desktop test
	}
}
