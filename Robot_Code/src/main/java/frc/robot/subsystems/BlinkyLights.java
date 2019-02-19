package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.loops.Looper;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;

public class BlinkyLights extends Subsystem {
    private static final BlinkyLights m_lights = new BlinkyLights();
    private double LightPower;
    private int LightPowerINT;
    private Spark light;
    private String LightColour;
    private boolean TorF;
    public static BlinkyLights getInstance(){return m_lights;}
    private BlinkyLights() {
        light = new Spark(Constants.LIGHTS);
        LightPower = .91;
        LightPowerINT = 0;
        LightColour = "Fail";
        TorF = false;
    }
    public Loop mloop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            LightPower = 0.91;
        }
        @Override
        public void onLoop(double timestamp) {
            if (TorF) {
                LightPower = 0.61;
            } else {
                LightPower = 0.91;
            }
            LightPowerINT = (int) LightPower * 100;
            switch (LightPowerINT) {
                case 61: LightColour = "Red"; break;
                case 87: LightColour = "Blue"; break;
                case 73: LightColour = "Green"; break;
                default: LightColour = "Error"; break;
            }
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
        light.setInverted(false);
        light.setSpeed(LightPower);
        light.set(LightPower);
        SmartDashboard.putString("Color", LightColour);
        SmartDashboard.putNumber("Power", LightPower);
        SmartDashboard.putNumber("# Spark is Getting", light.getSpeed());
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void reset() {

    }


    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(mloop);
    }
}
