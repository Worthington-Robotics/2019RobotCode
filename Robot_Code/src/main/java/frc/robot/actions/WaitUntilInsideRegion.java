package frc.robot.actions;

import frc.lib.geometry.Translation2d;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.PoseEstimator;

public class WaitUntilInsideRegion extends Action {
    private final static PoseEstimator mRobotState = PoseEstimator.getInstance();

    private final Translation2d mBottomLeft;
    private final Translation2d mTopRight;

    public WaitUntilInsideRegion(Translation2d bottomLeft, Translation2d topRight, boolean isOnLeft) {
        if (isOnLeft) {
            mBottomLeft = new Translation2d(bottomLeft.x(), -topRight.y());
            mTopRight = new Translation2d(topRight.x(), -bottomLeft.y());
        } else {
            mBottomLeft = bottomLeft;
            mTopRight = topRight;
        }
    }

    @Override
    public boolean isFinished() {
        Translation2d position = mRobotState.getLatestFieldToVehicle().getValue().getTranslation();
        return position.x() > mBottomLeft.x() && position.x() < mTopRight.x()
                && position.y() > mBottomLeft.y() && position.y() < mTopRight.y();
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
