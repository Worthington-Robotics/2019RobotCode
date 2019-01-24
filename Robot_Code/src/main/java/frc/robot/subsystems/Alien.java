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
        AlienOne = new DoubleSolenoid(Constants.ALIEN_1_LOW_ID,Constants.ALIEN_1_HIGH_ID);
        AlienTwo = new DoubleSolenoid(Constants.ALIEN_2_LOW_ID,Constants.ALIEN_2_HIGH_ID);

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
        private DoubleSolenoid.Value tState = DoubleSolenoid.Value.kOff;
        public boolean AlienTopLimit = false;
        public boolean AlienBottomLimit = false;
    }
}