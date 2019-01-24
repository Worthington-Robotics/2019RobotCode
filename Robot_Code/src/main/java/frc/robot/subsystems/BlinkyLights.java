package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.Loop;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;

public class BlinkyLights extends Subsystem {
    private static final BlinkyLights m_lights = new BlinkyLights();
    private double LightPower;
    private int LightPowerINT;
    private Spark light;
    private DriverStation.Alliance color;
    private String LightColour;
    private BlinkyLights() {
        light = new Spark(Constants.LIGHTS);
        LightPower = 0;
        LightPowerINT = 0;
        LightColour = "Fail";
        color = DriverStation.getInstance().getAlliance();
    }

    public static BlinkyLights getInstance(){return m_lights;}
    @Override
    public void readPeriodicInputs() {
        switch (color) {
            case Red: LightPower = .61; break;
            case Blue: LightPower = .87; break;
            case Invalid: LightPower = .93; break;
            default: LightPower = .99; break;
        }
        LightPowerINT = (int) (LightPower * 100);
        switch (LightPowerINT) {
            case 61: LightColour = "Red"; break;
            case 87: LightColour = "Blue"; break;
            case 93: LightColour = "White/Invalid"; break;
            default: LightColour = "Um wot?"; break;
        }
    }

    @Override
    public void writePeriodicOutputs() {
        light.set(LightPower);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Color", LightColour);
    }

    @Override
    public void stop() {

    }

    @Override
    public void reset() {

    }
}
