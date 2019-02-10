package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Constants;

/**
 * The Alien Subsystem manipulates the hatch panels with
 * a pair of pistons controlled with a central double solenoid
 */
public class Alien extends Subsystem {
    private static Alien m_Alien = new Alien();

    /**
     * The getter of the only public Alien instance
     *
     * @return the only instance of Alien to maintain a clean system
     */
    public static Alien getInstance() {
        return m_Alien;
    }



    private DoubleSolenoid AlienOne;
    private PeriodicIO periodic;

    private Alien() {
        AlienOne = new DoubleSolenoid(Constants.ALIEN_1_LOW_ID, Constants.ALIEN_1_HIGH_ID);
        periodic = new PeriodicIO();
        periodic.tState = DoubleSolenoid.Value.kForward;
    }


    public void outputTelemetry() {

    }

    public void readPeriodicInputs() {
    }

    public void writePeriodicOutputs() {
        AlienOne.set(periodic.tState);

    }

    /**
     * sets the solenoid to a DoubleSolenoid.Value state
     *
     * @param state either kOff kForward or kReverse
     */
    public void setAlienState(DoubleSolenoid.Value state) {
        periodic.tState = state;
    }

    /**
     * gets the solenoid to a DoubleSolenoid.Value state
     *
     * @return either kOff kForward or kReverse
     */
    public DoubleSolenoid.Value GetAlienState() {
        return periodic.tState;
    }


    public void reset() {
        periodic.tState = DoubleSolenoid.Value.kForward;
    }

    /**
     * stores regularly updated values
     */
    public class PeriodicIO {
        DoubleSolenoid.Value tState = DoubleSolenoid.Value.kForward;
    }
}