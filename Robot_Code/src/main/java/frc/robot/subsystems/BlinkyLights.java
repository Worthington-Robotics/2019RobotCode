package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.Loop;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;

public class BlinkyLights extends Subsystem {
    private static final BlinkyLights m_lights = new BlinkyLights();
    private double LightPower;
    private Spark light;
    private BlinkyLights() {
        light = new Spark(Constants.LIGHTS);
        LightPower = 0;
    }

    public static BlinkyLights getInstance(){return m_lights;}
    @Override
    public void readPeriodicInputs() {
        LightPower =  (SmartDashboard.getNumber("DB/Slider 0", 2.5)-2.5);
    }

    @Override
    public void writePeriodicOutputs() {
        light.set(LightPower);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Pattern #", LightPower);
    }

    @Override
    public void stop() {

    }

    @Override
    public void reset() {

    }
}
