package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.loops.ILooper;
import frc.robot.Constants;

public class Alien extends Subsystem {

    private static Alien m_Alien = new Alien();
    private DoubleSolenoid AlienOne;
    private DoubleSolenoid AlienTwo;
    private PeriodicIO Periodic;

    public Alien() {
        AlienOne = new DoubleSolenoid(0,1);
        AlienTwo = new DoubleSolenoid(2,3);

        PeriodicIO Periodic = new PeriodicIO();
    }

    public static Alien getInstance() {
        return m_Alien;
    }

    @Override
    public void outputTelemetry() {

    }

    public void readPeriodicInputs() {
    }

    public void writePeriodicOutputs() {
        AlienOne.set(Periodic.tState);
        AlienTwo.set(Periodic.tState);
    }


    @Override
    public void stop() {
        Periodic.tState = DoubleSolenoid.Value.kOff;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {

    }

    public void setAlienState(DoubleSolenoid.Value state) {
        Periodic.tState = state;
    }

    public DoubleSolenoid.Value GetAlienState() {
        return Periodic.tState;
    }

    @Override
    public void reset() {
        Periodic.tState = DoubleSolenoid.Value.kOff;
    }

    public class PeriodicIO {
        private DoubleSolenoid.Value tState;
        public boolean AlienTopLimit = false;
        public boolean AlienBottomLimit = false;
    }
}