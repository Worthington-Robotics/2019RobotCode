package frc.robot.actions.driveactions;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.statemachine.Action;
import frc.lib.util.DriveSignal;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.vision.AnglePIDOutput;
import frc.robot.subsystems.vision.AnglePIDSource;

public class AnglePID extends Action {
    private PIDController angleController;
    private AnglePIDSource source = new AnglePIDSource();
    private AnglePIDOutput output = new AnglePIDOutput();

    @Override
    public void onStart() {
        double angleOffset = SmartDashboard.getNumber("vision/angleOffset", -1000);

        if (angleOffset >= -180 && angleOffset <= 180) {
            double currentAngle = Drive.getInstance().getHeading().getDegrees();
            double desiredAngle = (angleOffset + currentAngle);
            if (desiredAngle > 180) {
                desiredAngle -= 360;
            } else if (desiredAngle < -180) {
                desiredAngle += 360;
            }

            SmartDashboard.putNumber("vision/Start Angle", currentAngle);
            SmartDashboard.putNumber("vision/Start Angle Offset", angleOffset);
            SmartDashboard.putNumber("vision/Desired Angle", desiredAngle);

            angleController = new PIDController(Constants.ANGLE_KP, Constants.ANGLE_KI, Constants.ANGLE_KD, source, output);
            angleController.setSetpoint(desiredAngle);
            angleController.setAbsoluteTolerance(1.0);
            angleController.setOutputRange(-0.5, 0.5);
            angleController.setInputRange(-180, 180);
            angleController.setContinuous();
            angleController.enable();
        }
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
