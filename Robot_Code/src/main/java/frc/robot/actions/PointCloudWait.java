package frc.robot.actions;

import frc.lib.geometry.Pose2d;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.PoseEstimator;

public class PointCloudWait extends Action {
    private boolean isX, isY, isTheta;
    private double epsilonX, epsilonY, epsilonTheta, X, Y, Theta;
    public PointCloudWait(Pose2d pose, double epsilonX, double epsilonY, double epsilonTheta)
    {
        Theta = pose.getRotation().getDegrees();
        X = pose.getTranslation().x();
        Y = pose.getTranslation().y();
        this.epsilonX = epsilonX;
        this.epsilonY = epsilonY;
        this.epsilonTheta = epsilonTheta;
    }
    @Override
    public void onStart() {
        isX = false;
        isY = false;
        isTheta = false;

    }

    public void onLoop() {
        double mX = PoseEstimator.getInstance().getPoseX();
        double mY = PoseEstimator.getInstance().getPoseY();
        double mTheta = PoseEstimator.getInstance().getPoseTheta();
        isX = Math.abs(X-mX) <= epsilonX;
        isY = Math.abs(Y-mY) <= epsilonY;
        isTheta = Math.abs(Theta-mTheta) <= epsilonTheta;
    }

    public boolean isFinished() {
        return (isX && isY)&& isTheta;
    }

    public void onStop() {

    }
}
