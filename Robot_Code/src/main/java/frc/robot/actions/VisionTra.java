package frc.robot.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.statemachine.Action;
import frc.lib.trajectory.TimedView;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.TrajectoryIterator;
import frc.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.util.Ultrasonic;
import frc.robot.planners.TraGenerator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PoseEstimator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class VisionTra extends Action {
    Trajectory<TimedState<Pose2dWithCurvature>> visionTra;
    private double distance;
    private List<Pose2d> Points;
    private static final double minDist = 3;

    @Override
    public void onStart() {
        Pose2d currentPose = new Pose2d(PoseEstimator.getInstance().getLatestFieldToVehicle().getValue());
        distance = Arm.getInstance().getUltrasonicDistance();
        Points = new ArrayList<>();
        Points.add(currentPose);
        Points.add(new Pose2d(currentPose.getTranslation().x() + distance*currentPose.getRotation().cos(),
                currentPose.getTranslation().y() + distance*currentPose.getRotation().sin(),
                currentPose.getRotation()));
        visionTra = TraGenerator.getInstance().generateTrajectory(false, Points,
                Arrays.asList(new CentripetalAccelerationConstraint(60)), 6,3,10);
        Drive.getInstance().setTrajectory(new TrajectoryIterator<>(new TimedView<>(visionTra)));
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        distance = Arm.getInstance().getUltrasonicDistance();
        return distance < minDist;
    }

    @Override
    public void onStop() {
        Drive.getInstance().overrideTrajectory(true);
    }
}
