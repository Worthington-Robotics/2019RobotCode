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
    private final Pose2d HabStart, RocketMidPoint, RocketApproch, Rocket, HatchPickup, LRocketMidPoint, LRocketApproch, LRocket, LHatchPickup;
    private final Pose2d HabOff, CargoShipMid, CargoHoldMid, Cargo, CargoShip, HairpinTurn;

    private DriveTrajectoryGenerator() {
        DMP = new DriveMotionPlanner();
        HabOff = new Pose2d(0, 20, Rotation2d.fromDegrees(0));
        Cargo = new Pose2d(0,60, Rotation2d.fromDegrees(135));
        HairpinTurn = new Pose2d(221,60, Rotation2d.fromDegrees(135));
        CargoHoldMid = new Pose2d(176,15,Rotation2d.fromDegrees(180));
        CargoShip = new Pose2d(191, 0, Rotation2d.fromDegrees(90));
        CargoShipMid = new Pose2d(131, 60, Rotation2d.fromDegrees(90));
        HabStart = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        RocketMidPoint = new Pose2d(-163, -48, Rotation2d.fromDegrees(0));
        RocketApproch = new Pose2d(-223, -96, Rotation2d.fromDegrees(-30));
        Rocket = new Pose2d(-185, -110, Rotation2d.fromDegrees(-30));
        HatchPickup = new Pose2d(48, -91, Rotation2d.fromDegrees(0));
        LRocketMidPoint = new Pose2d(-163, 48, Rotation2d.fromDegrees(0));
        LRocketApproch = new Pose2d(-223, 96, Rotation2d.fromDegrees(30));
        LRocket = new Pose2d(-185, 110, Rotation2d.fromDegrees(30));
        LHatchPickup = new Pose2d(48, 91, Rotation2d.fromDegrees(0));
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

    public Trajectory<TimedState<Pose2dWithCurvature>> HabToOffHab(boolean reversed) {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(HabStart);
        Points.add(HabOff);
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> OffHabToHab(boolean reversed) {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(HabOff);
        Points.add(HabStart);
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> CargoToCargoMid(boolean reversed) {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(Cargo);
        Points.add(CargoHoldMid);
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }


    public Trajectory<TimedState<Pose2dWithCurvature>> HabToCargoShipMid(boolean reversed) {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(HabStart);
        Points.add(CargoShipMid);
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> CargoShipMidToCargoShip(boolean reversed) {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(CargoShipMid);
        Points.add(CargoShip);
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> CargoMidToHairpin(boolean reversed) {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(CargoHoldMid);
        Points.add(HairpinTurn);
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> HairpinToCargoShip(boolean reversed) {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(HairpinTurn);
        Points.add(CargoShip);
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> CargoShipToHairpin(boolean reversed) {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(CargoShip);
        Points.add(HairpinTurn);
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> HairpinToCargoMid(boolean reversed) {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(HairpinTurn);
        Points.add(CargoHoldMid);
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> CargoMidtoCargo(boolean reversed) {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(HabStart);
        Points.add(RocketMidPoint);
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
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
    public Trajectory<TimedState<Pose2dWithCurvature>> RevHatchPickupToRocketMidPoint() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(HatchPickup);
        Points.add(RocketMidPoint);
        return generateTrajectory(true, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> RevHabToLRocketMid() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(HabStart);
        Points.add(RocketMidPoint);
        return generateTrajectory(true, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> RevLRocketMidToLRocketApproch() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(RocketMidPoint);
        Points.add(RocketApproch);
        return generateTrajectory(true, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> LRocketApprochToLRocket() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(RocketApproch);
        Points.add(Rocket);
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> RevLRocketToLRocketApproch() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(Rocket);
        Points.add(RocketApproch);
        return generateTrajectory(true, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> LRocketApprochToLRocketMidpoint() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(RocketApproch);
        Points.add(RocketMidPoint);
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> LRocketMidPointToLHatchPickup() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(RocketMidPoint);
        Points.add(HatchPickup);
        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> RevLHatchPickupToLRocketMidPoint() {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(HatchPickup);
        Points.add(RocketMidPoint);
        return generateTrajectory(true, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }


}