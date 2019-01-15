package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;

public class CargoMani extends Subsystem {
    private static final CargoMani m_instance = new CargoMani();
    private PeriodicIO periodic;
    private Spark topMotor;
    private Spark bottomMotor;

    private CargoMani() {
        topMotor = new Spark(Constants.TOP_CARGOMANIP_ID);
        bottomMotor = new Spark(Constants.BOTTOM_CARGOMANIP_ID);

    }

    public static CargoMani getInstance() {
        return m_instance;
    }
    public void setShotPower (double Power) {periodic.ShotPower = Power;}

    @Override
    public void readPeriodicInputs() {
        double[] operatorstick = HIDHelper.getAdjStick(Constants.SECOND_STICK);
    }

     public void writePeriodicOutputs (){
               // topMotor.set(periodic.ShotPower);
               // bottomMotor.set(periodic.ShotPower);
    }

    public void outputTelemetry() {

    }

    public void stop() {

    }

    public void reset() {

    }
    public static class PeriodicIO {
        public double ShotPower = 0.0;
    }
}
