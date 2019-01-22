package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.Loop;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;

public class BlinkyLights extends Subsystem {
    public static final BlinkyLights m_lights = new BlinkyLights();
    private double LightPower;
    private Spark light;
    private BlinkyLights() {
        light = new Spark(Constants.LIGHTS);
        LightPower = 0;
    }

    public static BlinkyLights getInstance(){return m_lights;}

    private final Loop LightsLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {
            LightPower = HIDHelper.getAdjStick(Constants.MASTER_STICK)[1];
        }
        @Override
        public void onStop(double timestamp) {

        }
    };
    @Override
    public void readPeriodicInputs() {

    }

    @Override
    public void writePeriodicOutputs() {
        light.set(LightPower);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Light Number", LightPower);
    }

    @Override
    public void stop() {

    }

    @Override
    public void reset() {

    }
}
