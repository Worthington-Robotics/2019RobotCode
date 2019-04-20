package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.DriveSignal;
import frc.robot.subsystems.Drive;

public class AnglePIDOutput implements PIDOutput {

    private static final double MIN_OUTPUT = 0.31;
    private static final double THRESHOLD = 0.13;

    @Override
    public void pidWrite(double output) {
        boolean negative = output < 0.0;
        if (output != 0.0 && Math.abs(output) > THRESHOLD && Math.abs(output) < MIN_OUTPUT) {
            output = MIN_OUTPUT;
            if (negative) {
                output *= -1;
            }
        } else if (Math.abs(output) <= THRESHOLD) {
            output = 0;
        }

        DriveSignal signal = new DriveSignal(output, -output, false);
        SmartDashboard.putNumber("vision/Drive Signal Output", output);
        Drive.getInstance().setAnglePidLoop(signal);
    }
}
