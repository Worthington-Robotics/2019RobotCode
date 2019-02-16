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
    //private Ultrasonic US = new Ultrasonic(1, 0);
    private List<Pose2d> Points;
    private static final double minDist = 8;
    private static final double maxDist = 100;

    public VisionTra() {
        //US.setAutomaticMode(true);
        //US.setEnabled(true);
    }

    public void onStart() {
        Pose2d currentPose = new Pose2d(PoseEstimator.getInstance().getLatestFieldToVehicle().getValue());
        distance = Arm.getInstance().getUltrasonicDistance();
        Points = new ArrayList<>();
        Points.add(currentPose);
        if (distance > 0) {
            Pose2d targetPose = new Pose2d(currentPose.getTranslation().x() + distance * currentPose.getRotation().cos(),
                    currentPose.getTranslation().y() + distance * currentPose.getRotation().sin(), currentPose.getRotation());
            Points.add(targetPose);

            visionTra = TraGenerator.getInstance().generateTrajectory(false, Points,
                    Arrays.asList(new CentripetalAccelerationConstraint(60)), 24, 12, 10);
            Drive.getInstance().setTrajectory(new TrajectoryIterator<>(new TimedView<>(visionTra)));

            SmartDashboard.putNumber("ultrasonic/Start Distance", distance);
            SmartDashboard.putNumber("ultrasonic/Start Pose X", currentPose.getTranslation().x());
            SmartDashboard.putNumber("ultrasonic/Start Pose Y", currentPose.getTranslation().y());
            SmartDashboard.putNumber("ultrasonic/Target Pose X", targetPose.getTranslation().x());
            SmartDashboard.putNumber("ultrasonic/Target Pose Y", targetPose.getTranslation().y());
        }
    }

    public void onLoop() {
        distance = Arm.getInstance().getUltrasonicDistance();
        SmartDashboard.putNumber("ultrasonic/Distance", distance);
    }

    public boolean isFinished() {
        return distance < minDist || distance > maxDist;
    }

    public void onStop() {
        Drive.getInstance().overrideTrajectory(true);
    }
}
