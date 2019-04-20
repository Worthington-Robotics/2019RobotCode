package frc.robot.actions.driveactions;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.statemachine.Action;
import frc.lib.util.DriveSignal;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.vision.AnglePIDSource;
import frc.robot.subsystems.vision.GyroPIDOutput;

public class GyroLock extends Action {

    private PIDController angleController;
    private AnglePIDSource source = new AnglePIDSource();
    private GyroPIDOutput output = new GyroPIDOutput();

    @Override
    public void onStart() {
        angleController = new PIDController(Constants.ANGLE_KP, Constants.ANGLE_KI, Constants.ANGLE_KD, source, output);
        angleController.setSetpoint(Drive.getInstance().getHeading().getDegrees());
        angleController.setAbsoluteTolerance(1.0);
        angleController.setOutputRange(-0.5, 0.5);
        angleController.setInputRange(-180, 180);
        angleController.setContinuous();
        angleController.enable();
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        if (angleController != null) {
            SmartDashboard.putBoolean("vision/On Target", angleController.onTarget());
            // return angleController.onTarget();
            return false;
        }
        return true;
    }

    @Override
    public void onStop() {

        if (angleController != null) {
            angleController.disable();
            angleController.close();
        }
        Drive.getInstance().setOpenLoop(new DriveSignal(0, 0));

        SmartDashboard.putNumber("vision/End Angle", Drive.getInstance().getHeading().getDegrees());

    }
}
