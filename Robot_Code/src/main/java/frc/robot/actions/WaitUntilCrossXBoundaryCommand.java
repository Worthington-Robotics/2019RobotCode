package frc.robot.actions;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PoseEstimator;

public class WaitUntilCrossXBoundaryCommand extends Action {
    private double mXBoundary = 0;

    public WaitUntilCrossXBoundaryCommand(double x) {
        mXBoundary = x;
    }

    @Override
    public boolean isFinished() {
        return PoseEstimator.getInstance().getLatestFieldToVehicle().getValue().getTranslation().x() > mXBoundary;
    }

    @Override
    public void onLoop() {

    }

    @Override
    public void onStop() {

    }

    @Override
    public void onStart() {

    }
}
