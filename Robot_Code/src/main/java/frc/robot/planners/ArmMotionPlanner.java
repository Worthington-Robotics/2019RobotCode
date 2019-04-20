package frc.robot.planners;

import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.physics.ArmModel;
import frc.lib.physics.DCMotorTransmission;
import frc.lib.trajectory.TimedView;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.TrajectoryIterator;
import frc.lib.trajectory.TrajectorySamplePoint;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.trajectory.timing.TimingConstraint;
import frc.lib.trajectory.timing.TimingUtil;
import frc.lib.util.CSVWritable;
import frc.robot.Constants;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

public class ArmMotionPlanner implements CSVWritable {

    private static final double MAX_DX = 0.1;
    private static final double MAX_DY = 0.1;
    private static final double MAX_DTHETA = Math.toRadians(5.0);

    private FollowerType followerType = FollowerType.FEED_FORWARD_ONLY;
    private Output output;
    private TimedState<Rotation2d> mProxSetpoint = new TimedState<>(Rotation2d.identity()), mDistSetpoint = new TimedState<>(Rotation2d.identity());
    private Translation2d error = Translation2d.identity();
    private TrajectoryIterator<TimedState<Rotation2d>> mCurrentProxTrajectory, mCurrentDistTrajectory;
    private double mDt = 0.0, lastTime = Double.POSITIVE_INFINITY;
    private ArmModel mModel;


    public ArmMotionPlanner() {
        final DCMotorTransmission proxTransmission = new DCMotorTransmission(1 / Constants.PROX_Kv, Constants.PROX_Kt, Constants.PROX_V_INTERCEPT); //TODO calculate odd ka value
        final DCMotorTransmission distTransmission = new DCMotorTransmission(1 / Constants.DIST_Kv, Constants.DIST_Kt, Constants.DIST_V_INTERCEPT); //TODO calculate odd ka value
        mModel = new ArmModel(Constants.PROX_MOI, Constants.DIST_MOI, proxTransmission, distTransmission);

    }

    public void reset() {
        error = Translation2d.identity();
        output = new Output();
        lastTime = Double.POSITIVE_INFINITY;
    }

    public void setFollowerType(FollowerType type) {
        followerType = type;
    }

    /*public static ArmTrajectory generateTrajectory(
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Rotation2d>> constraints,
            double start_vel,
            double end_vel,
            double max_vel,  // inches/s
            double max_accel ){  // inches/s^2
        final Trajectory<Translation2d> trajectory = TrajectoryUtil.pointTrajectoryFromSplineWaypoints(waypoints, MAX_DX, MAX_DY, MAX_DTHETA);
        final List<Rotation2d> prox_waypts = new ArrayList<>(), dist_waypts = new ArrayList<>();
        for(int i = 0; i < trajectory.length(); i++){
            final ArmModel.ArmState current = ArmModel.solveInverseKinematics(Units.inches_to_meters(trajectory.getPoint(i).state().getTranslation().x()),
                    Units.inches_to_meters(trajectory.getPoint(i).state().getTranslation().y()));
            if(current.isNAN()) throw new RuntimeException("Calculated NAN for arm Configuration! check for invalid coordinates");
            prox_waypts.add(Rotation2d.fromRadians(current.prox));
            dist_waypts.add(Rotation2d.fromRadians(current.dist));
        }
        System.out.println();
        final Trajectory<TimedState<Rotation2d>> prox_timed = TimingUtil.timeParameterizeTrajectory(false, prox_waypts, constraints, start_vel, end_vel, max_vel, max_accel);
        final Trajectory<TimedState<Rotation2d>> dist_timed = TimingUtil.timeParameterizeTrajectory(false, dist_waypts, constraints, start_vel, end_vel, max_vel, max_accel );
        return new ArmTrajectory(prox_timed, dist_timed);
    }*/

    public static ArmTrajectory generateTrajectory(
            final List<Rotation2d> proxWaypoints,
            final List<Rotation2d> distWaypoints,
            final List<TimingConstraint<Rotation2d>> constraints,
            double start_vel,
            double end_vel,
            double max_vel,  // inches/s
            double max_accel) {  // inches/s^2

        final Trajectory<TimedState<Rotation2d>> prox_timed = TimingUtil.timeParameterizeTrajectory(false, interpoltate(proxWaypoints, MAX_DTHETA), constraints, start_vel, end_vel, max_vel, max_accel);
        final Trajectory<TimedState<Rotation2d>> dist_timed = TimingUtil.timeParameterizeTrajectory(false, interpoltate(distWaypoints, MAX_DTHETA), constraints, start_vel, end_vel, max_vel, max_accel);
        return new ArmTrajectory(prox_timed, dist_timed);
    }

    public void setTrajectory(final ArmTrajectory trajectory) {
        mCurrentProxTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory.proxTraj));
        mProxSetpoint = trajectory.proxTraj.getFirstState();
        mCurrentDistTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory.distTraj));
        mDistSetpoint = trajectory.distTraj.getFirstState();
    }

    public Output update(double timestamp, Rotation2d prox, Rotation2d dist) {
        if (mCurrentProxTrajectory == null) return new Output();
        if (mCurrentProxTrajectory.getProgress() == 0.0 && !Double.isFinite(lastTime)) lastTime = timestamp;
        mDt = timestamp - lastTime;
        lastTime = timestamp;
        TrajectorySamplePoint<TimedState<Rotation2d>> prox_sample_point = mCurrentProxTrajectory.advance(mDt);
        mProxSetpoint = prox_sample_point.state();
        TrajectorySamplePoint<TimedState<Rotation2d>> dist_sample_point = mCurrentDistTrajectory.advance(mDt);
        mDistSetpoint = dist_sample_point.state();
        if (!mCurrentProxTrajectory.isDone() && !mCurrentDistTrajectory.isDone()) {
            // calculate model update
            final ArmModel.ArmState position = new ArmModel.ArmState(mProxSetpoint.state().getRadians(), mDistSetpoint.state().getRadians());
            final ArmModel.ArmState velocity = new ArmModel.ArmState(mProxSetpoint.velocity(), mDistSetpoint.velocity());
            final ArmModel.ArmState accel = new ArmModel.ArmState(mProxSetpoint.acceleration(), mDistSetpoint.acceleration());
            final ArmModel.ArmDynamics dynamics = mModel.solveInverseDynamics(position, velocity, accel);
            error = ArmModel.solveForwardKinematics(mProxSetpoint.state(), mDistSetpoint.state()).inverse().translateBy(ArmModel.solveForwardKinematics(prox, dist));
            //System.out.println(dynamics.toString());
            switch (followerType) {
                case FEEDBACK_CONTROLER:
                    //TODO implement feedback controller
                    output = new Output();
                    break;

                case FEED_FORWARD_ONLY:
                    output = new Output(dynamics.angular_velocity.prox, dynamics.angular_acceleration.prox, dynamics.voltage.prox,
                            dynamics.angular_velocity.dist, dynamics.angular_acceleration.dist, dynamics.voltage.dist);
                    //TODO check

            }

        } else {
            // TODO switch  arm to a pose stabilizing controller
            output = new Output();
        }
        return output;
    }


    public static class Output {

        public double proxVel = 0.0, proxAccel = 0.0, proxVoltage = 0.0;
        public double distVel = 0.0, distAccel = 0.0, distVoltage = 0.0;

        public Output(double proxVel, double proxAccel, double proxVoltage, double distVel, double distAccel, double distVoltage) {
            this.proxVel = proxVel;
            this.proxAccel = proxAccel;
            this.proxVoltage = proxVoltage;
            this.distVel = distVel;
            this.distAccel = distAccel;
            this.distVoltage = distVoltage;
        }

        public Output() {
        }

    }

    public static class ArmTrajectory implements CSVWritable {
        private Trajectory<TimedState<Rotation2d>> proxTraj, distTraj;

        public ArmTrajectory(Trajectory<TimedState<Rotation2d>> proxTraj, Trajectory<TimedState<Rotation2d>> distTraj) {
            this.proxTraj = proxTraj;
            this.distTraj = distTraj;
        }


        @Override
        public String toCSV() {
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < proxTraj.length(); i++) {
                sb.append(proxTraj.getState(i).toCSV());
                sb.append(", ");
                sb.append(distTraj.getState(i).toCSV());
                sb.append(System.lineSeparator());
            }
            return sb.toString();
        }

        public String toString() {
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < proxTraj.length(); i++) {
                sb.append("Prox: ");
                sb.append(proxTraj.getState(i).toString());
                sb.append(" Dist: ");
                sb.append(distTraj.getState(i).toString());
                sb.append(" Reconstructed pose: ");
                sb.append(ArmModel.solveForwardKinematics(proxTraj.getState(i).state(), distTraj.getState(i).state()).scale(39.3701).toString());
                sb.append(System.lineSeparator());
            }
            return sb.toString();
        }
    }

    public String toCSV() {
        DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(output.proxVel) + "," + fmt.format(output.distVel) + "," + fmt.format
                (output.proxVoltage) + "," + fmt.format(output.distVoltage)/* + "," +
                mProxSetpoint.toCSV()*/;
    }


    enum FollowerType {
        FEEDBACK_CONTROLER,
        FEED_FORWARD_ONLY
    }

    public boolean isDone() {
        return mCurrentProxTrajectory != null && mCurrentProxTrajectory.isDone() &&
                mCurrentDistTrajectory != null && mCurrentDistTrajectory.isDone();
    }

    public Translation2d error() {
        return error;
    }

    public Translation2d setpoint() {
        return ArmModel.solveForwardKinematics(mProxSetpoint.state(), mDistSetpoint.state()).scale(39.3701);
    }

    public TimedState<Rotation2d> proximalSetpoint() {
        return mProxSetpoint;
    }

    public TimedState<Rotation2d> distalSetpoint() {
        return mDistSetpoint;
    }

    private static ArrayList<Rotation2d> interpoltate(List<Rotation2d> x, double y) {
        ArrayList<Rotation2d> a = new ArrayList<>();
        for (int i = 0; i < x.size() - 1; i++) {
            double z = y / Math.abs(x.get(i).getDegrees() - x.get(i + 1).getDegrees());
            for (double j = 0; j <= 1; j += z) {
                a.add(x.get(i).interpolate(x.get(i + 1), j));
            }
        }
        a.add(x.get(x.size() - 1));
        return a;
    }

}