package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.Constants;

public class Manipulator extends Subsystem {
    private static final Manipulator m_instance = new Manipulator();
    private Spark bottomMotor, topMotor;
    private DoubleSolenoid ClimbFront, ClimbBack;
    private double ShotPower = 0.0;
    private DoubleSolenoid.Value fState = DoubleSolenoid.Value.kReverse;
    private DoubleSolenoid.Value bState = DoubleSolenoid.Value.kReverse;


    public Manipulator() {
        bottomMotor = new Spark(Constants.BOTTOM_CARGOMANIP_ID);
        topMotor = new Spark(Constants.TOP_CARGOMANIP_ID);
        ClimbFront = new DoubleSolenoid(Constants.CLIMB_FRONT_LOW_ID, Constants.CLIMB_FRONT_HIGH_ID);
        ClimbBack = new DoubleSolenoid(Constants.CLIMB_BACK_LOW_ID, Constants.CLIMB_BACK_HIGH_ID);
        reset();
    }

    public static Manipulator getInstance() {
        return m_instance;
    }

    public void setFrontState(DoubleSolenoid.Value state) {
        fState = state;
    }

    public DoubleSolenoid.Value GetFrontState() {
        return fState;
    }

    public void setBackState(DoubleSolenoid.Value state) {
        bState = state;
    }

    public DoubleSolenoid.Value GetBackState() {
        return bState;
    }

    public void setShotPower (double Power) {ShotPower = Power;}

     public void writePeriodicOutputs (){
        bottomMotor.set(-ShotPower);
        topMotor.set(-ShotPower);
        ClimbFront.set(fState);
        ClimbBack.set(bState);
    }

    public void outputTelemetry() {
    }

    public void reset() {
        topMotor.set(0);
        bottomMotor.set(0);
        ClimbBack.set(DoubleSolenoid.Value.kOff);
        ClimbFront.set(DoubleSolenoid.Value.kOff);
    }
}
