package frc.robot.planners;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.trajectory.timing.TimingConstraint;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class DriveTrajectoryGenerator {
    private static final DriveTrajectoryGenerator m_instance = new DriveTrajectoryGenerator();
    private final DriveMotionPlanner DMP;
    private Pose2d HabStart, RocketMidPoint, RocketApproch, Rocket, HatchPickup;

    private DriveTrajectoryGenerator() {
        DMP = new DriveMotionPlanner();
        HabStart = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        RocketMidPoint = new Pose2d(-163, -48, Rotation2d.fromDegrees(0));
        RocketApproch = new Pose2d(-223, -96, Rotation2d.fromDegrees(-30));
        Rocket = new Pose2d(-185, -110, Rotation2d.fromDegrees(-30));
        HatchPickup = new Pose2d(48, -91, Rotation2d.fromDegrees(0));
    }

    public static DriveTrajectoryGenerator getInstance() {
        return m_instance;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return DMP.generateTrajectory(reversed, waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> getTenFeet() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.identity()));
        Points.add(new Pose2d(120, 0, Rotation2d.fromDegrees(0)));
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> RevHabToRocketMid() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(HabStart);
        Points.add(RocketMidPoint);
        return generateTrajectory(true, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> RevRocketMidToRocketApproch() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(RocketMidPoint);
        Points.add(RocketApproch);
        return generateTrajectory(true, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> RocketApprochToRocket() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(RocketApproch);
        Points.add(Rocket);
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> RevRocketToRocketApproch() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(Rocket);
        Points.add(RocketApproch);
        return generateTrajectory(true, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> RocketApprochToRocketMidpoint() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(RocketApproch);
        Points.add(RocketMidPoint);
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> RocketMidPointToHatchPickup() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(RocketMidPoint);
        Points.add(HatchPickup);
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> RevRocketMidPointToHatchPickup() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(RocketMidPoint);
        Points.add(HatchPickup);
        return generateTrajectory(true, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }


}