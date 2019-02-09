package frc.robot.actions;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.vision.AnglePIDOutput;
import frc.robot.subsystems.vision.AnglePIDSource;

public class AnglePID extends Action {
    private double angleOffset, desiredAngle, currentAngle;
    private PIDController angleController;
    private AnglePIDSource source = new AnglePIDSource();
    private AnglePIDOutput output = new AnglePIDOutput();

    @Override
    public void onStart() {
        angleOffset = SmartDashboard.getNumber("vision/angleOffset", 0);
        SmartDashboard.putNumber("Angle Offset From Vision", angleOffset);
        currentAngle = Drive.getInstance().getHeading().getDegrees();
        desiredAngle = angleOffset + currentAngle;
        SmartDashboard.putNumber("Desired Angle", desiredAngle);
        angleController = new PIDController(Constants.ANGLE_KP, Constants.ANGLE_KI, Constants.ANGLE_KD, source, output);
        angleController.setSetpoint(desiredAngle);
        // angleController.enable();
    }

    @Override
    public void onLoop() {
    }

    @Override
    public boolean isFinished() {
        // return angleController.onTarget();
        return true;
    }

    @Override
    public void onStop() {
        angleController.disable();
    }

}
