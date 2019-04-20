package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.PIDOutput;
import frc.lib.util.DriveSignal;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class GyroPIDOutput implements PIDOutput {

    private static final double MIN_OUTPUT = 0.0;
    private static final double THRESHOLD = 0.0;

    @Override
    public void pidWrite(double output) {
        double drive = HIDHelper.getAdjStick(Constants.MASTER_STICK)[1];
        boolean negative = output < 0.0;
        if (output != 0.0 && Math.abs(output) > THRESHOLD && Math.abs(output) < MIN_OUTPUT) {
            output = MIN_OUTPUT;
            if (negative) {
                output *= -1;
            }
        } else if (Math.abs(output) <= THRESHOLD) {
            output = 0;
        }

        DriveSignal signal = new DriveSignal(output + (drive * 1), -output + (drive * 1), false);
        Drive.getInstance().setAnglePidLoop(signal);
    }
}
