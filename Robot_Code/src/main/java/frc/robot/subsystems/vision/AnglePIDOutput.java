package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.DriveSignal;
import frc.robot.subsystems.Drive;

public class AnglePIDOutput implements PIDOutput {
    @Override
    public void pidWrite(double output) {

        DriveSignal signal = new DriveSignal(output, -output, false);

        SmartDashboard.putNumber("vision/Drive Signal Output" , output);
        Drive.getInstance().setAnglePidLoop(signal);
    }
}
