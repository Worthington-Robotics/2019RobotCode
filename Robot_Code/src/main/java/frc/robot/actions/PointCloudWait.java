package frc.robot.actions;

import frc.lib.geometry.Pose2d;
import frc.lib.statemachine.Action;

public class PointCloudWait extends Action {
    private boolean end = false;
    private double epsilonX, epsilonY, epsilonTheta, X, Y, Theta;
    public PointCloudWait(Pose2d pose, double epsilonX, double epsilonY, double epsilonTheta)
    {
        Theta = pose.getRotation().getDegrees();
        X = pose.getTranslation().x();
        Y = pose.getTranslation().y();
    }
    @Override
    public void onStart() {

    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {

    }
}
