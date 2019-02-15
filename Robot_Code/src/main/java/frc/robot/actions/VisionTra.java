package frc.robot.actions;

import edu.wpi.first.wpilibj.Ultrasonic;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.statemachine.Action;
import frc.lib.trajectory.TimedView;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.TrajectoryIterator;
import frc.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.lib.trajectory.timing.TimedState;
import frc.robot.planners.TraGenerator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PoseEstimator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class VisionTra extends Action {
    Trajectory<TimedState<Pose2dWithCurvature>> visionTra;
    private double distance;
    private Ultrasonic US;
    private List<Pose2d> Points;
    private static final double minDist = 8;

    public void onStart() {
        US = new Ultrasonic(1,0);
        Pose2d currentPose = new Pose2d(PoseEstimator.getInstance().getLatestFieldToVehicle().getValue());
        distance = US.getRangeInches();
        Points = new ArrayList<>();
        Points.add(currentPose);
        Points.add(new Pose2d(currentPose.getTranslation().x() + distance*currentPose.getRotation().cos(),
                currentPose.getTranslation().y() + distance*currentPose.getRotation().sin(), currentPose.getRotation()));
        visionTra = TraGenerator.getInstance().generateTrajectory(false, Points,
                Arrays.asList(new CentripetalAccelerationConstraint(60)), 6,3,10);
        Drive.getInstance().setTrajectory(new TrajectoryIterator<>(new TimedView<>(visionTra)));
    }

    public void onLoop() {

    }

    public boolean isFinished() {
        distance = US.getRangeInches();
        return distance < minDist;
    }

    public void onStop() {
        Drive.getInstance().overrideTrajectory(true);
    }
}
