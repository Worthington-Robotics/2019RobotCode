package frc.robot.actions;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.statemachine.Action;
import frc.lib.util.DriveSignal;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.vision.AnglePIDOutput;
import frc.robot.subsystems.vision.AnglePIDSource;

public class AnglePID extends Action {
    private double angleOffset, desiredAngle;
    private PIDController angleController;
    private AnglePIDSource source = new AnglePIDSource();
    private AnglePIDOutput output = new AnglePIDOutput();

    @Override
    public void onStart() {
        angleOffset = SmartDashboard.getNumber("vision/angleOffset", 0);
        SmartDashboard.putNumber("Angle Offset From Vision", angleOffset);

        if (angleOffset >= -180 && angleOffset <= 180) {
            double currentAngle = Drive.getInstance().getHeading().getDegrees();
            desiredAngle = (angleOffset + currentAngle);
            if (desiredAngle > 180) {
                desiredAngle -= 360;
            }
            else if (desiredAngle < -180) {
                desiredAngle += 360;
            }
            SmartDashboard.putNumber("Desired Angle", desiredAngle);

            angleController = new PIDController(Constants.ANGLE_KP, Constants.ANGLE_KI, Constants.ANGLE_KD, source, output);
            angleController.setSetpoint(desiredAngle);
            angleController.setAbsoluteTolerance(0.5);
            angleController.setOutputRange(-0.6, 0.6);
            angleController.enable();
        }
    }

    @Override
    public void onLoop() {
        double currentAngle = Drive.getInstance().getHeading().getDegrees();
        SmartDashboard.putNumber("Current Angle" , currentAngle);
    }

    @Override
    public boolean isFinished() {
        return angleController.onTarget();
    }

    @Override
    public void onStop() {
        angleController.disable();
        Drive.getInstance().setOpenLoop(new DriveSignal(0, 0));
    }

}
