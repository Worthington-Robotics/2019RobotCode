package frc.robot.planners;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.trajectory.timing.TimingConstraint;

import java.util.List;

public class ArmTrajectoryGenerator {

    private static final ArmTrajectoryGenerator generator = new ArmTrajectoryGenerator();

    public ArmTrajectoryGenerator getInstance() {
        return generator;
    }

    private final ArmMotionPlanner AMP;

    private ArmTrajectoryGenerator() {
        AMP = new ArmMotionPlanner();
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel) {  // inches/s^2

        return null; //AMP.generateTrajectory(waypoints, constraints, 0.0, 0.0, max_vel, max_accel); //TODO FIX
    }


}
