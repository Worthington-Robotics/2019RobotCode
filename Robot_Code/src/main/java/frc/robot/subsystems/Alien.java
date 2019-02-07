package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.loops.ILooper;
import frc.robot.Constants;

public class Alien extends Subsystem {

    private static Alien m_Alien = new Alien();
    private DoubleSolenoid AlienOne;
    private DoubleSolenoid AlienTwo;
    private PeriodicIO periodic;

    public Alien() {
        AlienOne = new DoubleSolenoid(Constants.ALIEN_1_LOW_ID,Constants.ALIEN_1_HIGH_ID);
        periodic = new PeriodicIO();
        periodic.tState = DoubleSolenoid.Value.kForward;


    }

    public static Alien getInstance() {
        return m_Alien;
    }

    @Override
    public void outputTelemetry() {

    }

    public void readPeriodicInputs() {
    }

    public void writePeriodicOutputs(){
        AlienOne.set(periodic.tState);

    }


    @Override
    public void stop() {
        periodic.tState = DoubleSolenoid.Value.kForward;
    }

    public void setAlienState(DoubleSolenoid.Value state) {
        periodic.tState = state;
    }

    public DoubleSolenoid.Value GetAlienState() {
        return periodic.tState;
    }

    @Override
    public void reset() {
        periodic.tState = DoubleSolenoid.Value.kForward;
    }

    public class PeriodicIO {
        private DoubleSolenoid.Value tState = DoubleSolenoid.Value.kForward;
    }
}