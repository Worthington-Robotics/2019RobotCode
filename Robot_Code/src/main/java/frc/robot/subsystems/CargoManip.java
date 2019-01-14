package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import frc.robot.Constants;

public class CargoManip extends Subsystem   {
    Spark topMotor;
    Spark bottomMotor;
    private CargoManip() {
        topMotor = new Spark(Constants.TOP_CARGOMANIP_ID);
        bottomMotor = new Spark(Constants.BOTTOM_CARGOMANIP_ID);

    }

    public static CargoManip getInstance() {return m_instance; }
    public void setShotPower (double Power) {periodic.ShotPower = Power;}

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void reset() {

    }
}
