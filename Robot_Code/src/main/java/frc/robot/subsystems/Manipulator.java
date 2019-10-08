package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;

public class Manipulator extends Subsystem {
    private static final Manipulator m_instance = new Manipulator();
    private Spark bottomMotor, topMotor,climberCrawl, climberElevator;
    private DoubleSolenoid ClimbFront, Lock;
    private double ShotPower = 0.0;
    private boolean B2 = false;
    private double[] OperatorInput;
    private DoubleSolenoid.Value lState = DoubleSolenoid.Value.kForward;
    private DoubleSolenoid.Value fState = DoubleSolenoid.Value.kReverse;
    private double elevatorPower, crawlPower;


    public Manipulator() {
        bottomMotor = new Spark(Constants.BOTTOM_CARGOMANIP_ID);
        topMotor = new Spark(Constants.TOP_CARGOMANIP_ID);
        Lock = new DoubleSolenoid(Constants.CLIMB_FRONT_LOW_ID, Constants.CLIMB_FRONT_HIGH_ID);
        climberCrawl = new Spark(Constants.CLIMBER_CRAWL_ID);
        climberElevator = new Spark(Constants.CLIMBER_ELEVATOR_ID);
        ClimbFront = new DoubleSolenoid(Constants.LOCK_LOW_ID, Constants.LOCK_HIGH_ID);
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

    public void setLockState(DoubleSolenoid.Value state) {
        lState = state;
    }

    public DoubleSolenoid.Value GetBackState() {
        return lState;
    }

    public void setShotPower(double Power) {
        ShotPower = Power;
    }

    public void setCrawlPower(double Power) {
        crawlPower = Power;
    }

    public void setElevatorPower(double Power) {
        elevatorPower = Power;
    }

    public void readPeriodicInputs() {
        B2 = Constants.MASTER.getRawButton(2);
        OperatorInput = HIDHelper.getAdjStick(Constants.MASTER_STICK);

    }

    public void writePeriodicOutputs() {
        if (B2)
            climberCrawl.set(OperatorInput[1]);
        else
            climberCrawl.set(crawlPower);
        bottomMotor.set(-ShotPower);
        topMotor.set(-ShotPower);
        ClimbFront.set(fState);
        Lock.set(lState);
        climberElevator.set(elevatorPower);
    }

    public void outputTelemetry() {
    }

    public void reset() {
        climberElevator.set(0);
        climberCrawl.set(0);
        topMotor.set(0);
        bottomMotor.set(0);
        Lock.set(DoubleSolenoid.Value.kOff);
        ClimbFront.set(DoubleSolenoid.Value.kForward);
    }
}
