package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.Constants;

public class Manipulator extends Subsystem {
    private static final Manipulator m_instance = new Manipulator();
    private PeriodicIO periodic;
    private Spark topMotor, bottomMotor;
    private DoubleSolenoid AlienOne;


    public Manipulator() {
        topMotor = new Spark(Constants.TOP_CARGOMANIP_ID);
        bottomMotor = new Spark(Constants.BOTTOM_CARGOMANIP_ID);
        periodic = new PeriodicIO();
        AlienOne = new DoubleSolenoid(Constants.ALIEN_1_LOW_ID, Constants.ALIEN_1_HIGH_ID);
        periodic.tState = DoubleSolenoid.Value.kForward;
    }

    public static Manipulator getInstance() {
        return m_instance;
    }

    public void setAlienState(DoubleSolenoid.Value state) {
        periodic.tState = state;
    }

    public DoubleSolenoid.Value GetAlienState() {
        return periodic.tState;
    }

    public void setShotPower (double Power) {periodic.ShotPower = Power;}

     public void writePeriodicOutputs (){
        topMotor.set(periodic.ShotPower);
        bottomMotor.set(periodic.ShotPower);
        AlienOne.set(periodic.tState);
    }

    public void outputTelemetry() {

    }


    public void reset() {
        topMotor.set(0);
        bottomMotor.set(0);
        AlienOne.set(DoubleSolenoid.Value.kOff);
    }
    
    public static class PeriodicIO {
        public double ShotPower = 0.0;
        DoubleSolenoid.Value tState = DoubleSolenoid.Value.kForward;
    }
}
