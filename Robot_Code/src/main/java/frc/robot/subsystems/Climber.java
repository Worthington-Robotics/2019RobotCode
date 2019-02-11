package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import frc.robot.Constants;

public class Climber extends Subsystem {
    private static Climber m_Climber = new Climber();

    private Spark leftClime;
    private Spark rightClime;
    private boolean fullpower, antipower;
    private PeriodicIO periodic;


    private Climber() {
        leftClime = new Spark(Constants.LEFT_CLIME_ID);
        rightClime = new Spark(Constants.RIGHT_CLIME_ID);
        periodic = new PeriodicIO();

    }

    @Override
    public void readPeriodicInputs() {
        periodic.fullpower = Constants.LAUNCH_PAD.getRawButton(Constants.CLIMBER_FULL_POWER);
        if () {

        }
    }

    @Override
    public void writePeriodicOutputs() {

    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void reset() {

    }


    public class PeriodicIO {
        boolean fullpower = false;
        boolean antipower = false;
    }
}
