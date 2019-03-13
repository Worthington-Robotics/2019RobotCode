package frc.robot.planners;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class DriveTrajectoryGenerator {
    private static final DriveTrajectoryGenerator m_instance = new DriveTrajectoryGenerator();
    private final DriveMotionPlanner DMP;
    public final Pose2d HabStart;
    public final Pose2d HabOff, CargoShipMid, CargoHoldMid, Cargo, CargoShip1, CargoShip2, HairpinTurn, LCargoShipMid, LCargoHoldMid, LCargo, LCargoShip1, LCargoShip2, LHairpinTurn;

    private DriveTrajectoryGenerator() {
        DMP = new DriveMotionPlanner();
        HabOff       /**/ = new Pose2d(0, 20, Rotation2d.fromDegrees(0));
        Cargo        /**/ = new Pose2d(0, 60, Rotation2d.fromDegrees(-135));
        HairpinTurn  /**/ = new Pose2d(221, 60, Rotation2d.fromDegrees(-135));
        CargoHoldMid /**/ = new Pose2d(176, 45, Rotation2d.fromDegrees(-180));
        CargoShip1   /**/ = new Pose2d(197, 0, Rotation2d.fromDegrees(-90));
        CargoShip2   /**/ = new Pose2d(227, 0, Rotation2d.fromDegrees(-90));
        CargoShipMid /**/ = new Pose2d(131, 60, Rotation2d.fromDegrees(0));
        LCargo       /**/ = new Pose2d(0, -60, Rotation2d.fromDegrees(135));
        LHairpinTurn /**/ = new Pose2d(221, -60, Rotation2d.fromDegrees(135));
        LCargoHoldMid/**/ = new Pose2d(176, -45, Rotation2d.fromDegrees(180));
        LCargoShip1  /**/ = new Pose2d(197, 0, Rotation2d.fromDegrees(90));
        LCargoShip2  /**/ = new Pose2d(227, 0, Rotation2d.fromDegrees(90));
        LCargoShipMid/**/ = new Pose2d(131, -60, Rotation2d.fromDegrees(0));
        HabStart     /**/ = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        //
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

    public Trajectory<TimedState<Pose2dWithCurvature>> HabToOffHab(boolean reversed, boolean isRight) {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(HabStart);
        Points.add(HabOff);
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> OffHabToHab(boolean reversed, boolean isRight) {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(HabOff);
        Points.add(HabStart);
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> CargoToCargoMid(boolean reversed, boolean isRight) {
        List<Pose2d> Points = new ArrayList<>();
        if (isRight) {
            Points.add(Cargo);
            Points.add(CargoHoldMid);
        } else {
            Points.add(LCargo);
            Points.add(LCargoHoldMid);
        }
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }


    public Trajectory<TimedState<Pose2dWithCurvature>> HabToCargoShipMid(boolean reversed, boolean isRight) {
        List<Pose2d> Points = new ArrayList<>();
        if (isRight) {
            Points.add(HabStart);
            Points.add(CargoShipMid);
        } else {
            Points.add(HabStart);
            Points.add(LCargoShipMid);
        }
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> CargoShipMidToCargoShip(boolean reversed, boolean isRight, int bay) {
        List<Pose2d> Points = new ArrayList<>();
        switch (bay) {
            case 1:
                if (isRight) {
                    Points.add(CargoShipMid);
                    Points.add(CargoShip1);
                } else {
                    Points.add(LCargoShipMid);
                    Points.add(LCargoShip1);

                }
                break;
            case 2:
                if (isRight) {
                    Points.add(CargoShipMid);
                    Points.add(CargoShip2);
                } else {
                    Points.add(LCargoShipMid);
                    Points.add(LCargoShip2);

                }
                break;
        }
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> CargoMidToHairpin(boolean reversed, boolean isRight) {
        List<Pose2d> Points = new ArrayList<>();
        if (isRight) {
            Points.add(CargoHoldMid);
            Points.add(HairpinTurn);
        } else {
            Points.add(LCargoHoldMid);
            Points.add(LHairpinTurn);
        }
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> HairpinToCargoShip(boolean reversed, boolean isRight, int bay) {
        List<Pose2d> Points = new ArrayList<>();
        switch (bay) {
            case 1:
                if (isRight) {
                    Points.add(HairpinTurn);
                    Points.add(CargoShip1);
                } else {
                    Points.add(LHairpinTurn);
                    Points.add(LCargoShip1);
                }
                break;
            case 2:
                if (isRight) {
                    Points.add(HairpinTurn);
                    Points.add(CargoShip2);
                } else {
                    Points.add(LHairpinTurn);
                    Points.add(LCargoShip2);
                }
                break;
        }
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);

    }

    public Trajectory<TimedState<Pose2dWithCurvature>> CargoShipToHairpin(boolean reversed, boolean isRight, int bay) {
        List<Pose2d> Points = new ArrayList<>();
        switch (bay) {
            case 1:
                if (isRight) {
                    Points.add(CargoShip1);
                    Points.add(HairpinTurn);
                } else {
                    Points.add(LCargoShip1);
                    Points.add(LHairpinTurn);
                }
                break;
            case 2:
                if (isRight) {
                    Points.add(CargoShip2);
                    Points.add(HairpinTurn);
                } else {
                    Points.add(LCargoShip2);
                    Points.add(LHairpinTurn);
                }
                break;

        }
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> HairpinToCargoMid(boolean reversed, boolean isRight) {
        List<Pose2d> Points = new ArrayList<>();
        if (isRight) {
            Points.add(HairpinTurn);
            Points.add(CargoHoldMid);
        } else {
            Points.add(LHairpinTurn);
            Points.add(LCargoHoldMid);
        }
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> CargoMidtoCargo(boolean reversed, boolean isRight) {
        List<Pose2d> Points = new ArrayList<>();
        if (isRight) {
            Points.add(CargoHoldMid);
            Points.add(Cargo);
        } else {

            Points.add(LCargoHoldMid);
            Points.add(LCargo);
        }
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
}