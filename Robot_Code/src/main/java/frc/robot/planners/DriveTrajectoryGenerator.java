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

    private DriveTrajectoryGenerator() {
        DMP = new DriveMotionPlanner();
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

    public Trajectory<TimedState<Pose2dWithCurvature>> getRightRocket() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.identity()));
        Points.add(new Pose2d(108, 132, Rotation2d.fromDegrees(45)));
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36, 60, 10);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> autoTestProcedure() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        Points.add(new Pose2d(40, 0, Rotation2d.fromDegrees(0)));
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36, 60, 10);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> frontLeftCargoShipHatch() {
        List<Pose2d> Points = new ArrayList<>(); //140
        Points.add(new Pose2d(0,0,Rotation2d.identity()));
        Points.add(new Pose2d(140,0, Rotation2d.fromDegrees(0)));
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)),36,60,10);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> hatchLeftHighAndMid1() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0,0, Rotation2d.identity()));
        Points.add(new Pose2d(0,0 , Rotation2d.identity())); //TODO Fix Please
        return generateTrajectory(true,Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36,60,10);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> hatchLeftHighAndMid2() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0,0, Rotation2d.identity()));
        Points.add(new Pose2d(0,0, Rotation2d.identity()));
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)),  36, 60, 10);
    }public Trajectory<TimedState<Pose2dWithCurvature>> hatchLeftHighAndMid3() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0,0, Rotation2d.identity()));
        Points.add(new Pose2d(0,0, Rotation2d.identity()));
        return generateTrajectory(true, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)),  36, 60, 10);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> hatchRightHighAndMid1() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0,0, Rotation2d.identity()));
        Points.add(new Pose2d(0,0 , Rotation2d.identity())); //TODO Fix Please
        return generateTrajectory(true,Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36,60,10);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> hatchRightHighAndMid2() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0,0, Rotation2d.identity()));
        Points.add(new Pose2d(0,0, Rotation2d.identity()));
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)),  36, 60, 10);
    }public Trajectory<TimedState<Pose2dWithCurvature>> hatchRightHighAndMid3() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0,0, Rotation2d.identity()));
        Points.add(new Pose2d(0,0, Rotation2d.identity()));
        return generateTrajectory(true, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)),  36, 60, 10);
    }
}
