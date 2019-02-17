package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.Constants;

public class Manipulator extends Subsystem {
    private static final Manipulator m_instance = new Manipulator();
    private Spark topMotor, bottomMotor;
    private DoubleSolenoid AlienOne;

    private double ShotPower = 0.0;
    private DoubleSolenoid.Value tState = DoubleSolenoid.Value.kForward;


    public Manipulator() {
        topMotor = new Spark(Constants.TOP_CARGOMANIP_ID);
        bottomMotor = new Spark(Constants.BOTTOM_CARGOMANIP_ID);
        AlienOne = new DoubleSolenoid(Constants.ALIEN_1_LOW_ID, Constants.ALIEN_1_HIGH_ID);
        reset();
    }

    public static Manipulator getInstance() {
        return m_instance;
    }

    public void setAlienState(DoubleSolenoid.Value state) {
        tState = state;
    }

    public DoubleSolenoid.Value GetAlienState() {
        return tState;
    }

    public void setShotPower (double Power) {ShotPower = Power;}

     public void writePeriodicOutputs (){
        topMotor.set(ShotPower);
        bottomMotor.set(ShotPower);
        AlienOne.set(tState);
    }

    public void outputTelemetry() {

    }


    public void reset() {
        topMotor.set(0);
        bottomMotor.set(0);
        AlienOne.set(DoubleSolenoid.Value.kOff);
    }
}
