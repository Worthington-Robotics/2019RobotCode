package frc.lib.util;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalOutput;

public class Ultrasonic {

    private DigitalOutput outputPin;
    private Counter inputPin;
    private double distance;

    public Ultrasonic (int inputPin, int outputPin){
        this.outputPin = new DigitalOutput(outputPin);
        this.inputPin = new Counter(inputPin);
        this.inputPin.setSemiPeriodMode(true);
    }

    public void update(){
        outputPin.pulse(0.001); //1 ms pulse
        distance = inputPin.getPeriod() / 148;
    }

    public double getDistance(){
        update();
        return distance;
    }

}
