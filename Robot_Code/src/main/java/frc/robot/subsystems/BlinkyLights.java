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
    private Spark light;
    private DriverStation.Alliance color;
    private BlinkyLights() {
        light = new Spark(Constants.LIGHTS);
        LightPower = 0;
        color = DriverStation.getInstance().getAlliance();
    }

    public static BlinkyLights getInstance(){return m_lights;}
    @Override
    public void readPeriodicInputs() {
        if (color == DriverStation.Alliance.Blue) {
            LightPower = .87;
        } else if (color == DriverStation.Alliance.Red) {
            LightPower = .61;
        } else if (color == DriverStation.Alliance.Invalid) {
            LightPower = .93;
        }
        else { LightPower = .99; }
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
