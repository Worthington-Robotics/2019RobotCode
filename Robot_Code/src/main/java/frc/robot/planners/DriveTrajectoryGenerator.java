package frc.robot.planners;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.trajectory.timing.TimingConstraint;
import frc.lib.trajectory.timing.VelocityLimitRegionConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class DriveTrajectoryGenerator {
    private static final DriveTrajectoryGenerator m_instance = new DriveTrajectoryGenerator();
    private final DriveMotionPlanner DMP;
    public final Pose2d HabStart;
    public final Pose2d HabOff, CargoShipMid, CargoHoldMid, Cargo, CargoShip1, CargoShip2, HairpinTurn, RCargoShipMid, RCargoHoldMid, RCargo, RCargoShip1, RCargoShip2, RHairpinTurn;

    private DriveTrajectoryGenerator() {
        DMP          /**/ = new DriveMotionPlanner();
        HabOff       /**/ = new Pose2d(42, 0, Rotation2d.fromDegrees(0));
        HabStart     /**/ = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        Cargo        /**/ = new Pose2d(6, 33, Rotation2d.fromDegrees(-180));
        HairpinTurn  /**/ = new Pose2d(231, 65, Rotation2d.fromDegrees(-135));
        CargoHoldMid /**/ = new Pose2d(176, 35, Rotation2d.fromDegrees(-180));
        CargoShip1   /**/ = new Pose2d(197, -4, Rotation2d.fromDegrees(-90));
        CargoShip2   /**/ = new Pose2d(210, -4, Rotation2d.fromDegrees(-90));
        CargoShipMid /**/ = new Pose2d(131, 60, Rotation2d.fromDegrees(0));

        RCargo       /**/ = new Pose2d(6, -33, Rotation2d.fromDegrees(180));
        RHairpinTurn /**/ = new Pose2d(231, -65, Rotation2d.fromDegrees(135));
        RCargoHoldMid/**/ = new Pose2d(176, -35, Rotation2d.fromDegrees(180));
        RCargoShip1  /**/ = new Pose2d(197, 4, Rotation2d.fromDegrees(90));
        RCargoShip2  /**/ = new Pose2d(210, 4, Rotation2d.fromDegrees(90));
        RCargoShipMid/**/ = new Pose2d(131, -60, Rotation2d.fromDegrees(0));
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

    public Trajectory<TimedState<Pose2dWithCurvature>> HabToOffHab(boolean reversed) {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(HabStart);
        Points.add(HabOff);
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> OffHabToHab(boolean reversed) {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(HabOff);
        Points.add(new Pose2d(37, 0, Rotation2d.fromDegrees(0)));
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> CargoToCargoMid(boolean reversed, boolean isLeft) {
        List<Pose2d> Points = new ArrayList<>();
        if (isLeft) {
            Points.add(Cargo);
            Points.add(CargoHoldMid);
        } else {
            Points.add(RCargo);
            Points.add(RCargoHoldMid);
        }
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }



    public Trajectory<TimedState<Pose2dWithCurvature>> HabToCargoShipMid(boolean reversed, boolean isLeft) {
        List<Pose2d> Points = new ArrayList<>();
        if (isLeft) {
            Points.add(HabStart);
            Points.add(CargoShipMid);
        } else {
            Points.add(HabStart);
            Points.add(RCargoShipMid);
        }
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> CargoShipMidToCargoShip(boolean reversed, boolean isLeft, int bay) {
        List<Pose2d> Points = new ArrayList<>();
        switch (bay) {
            case 1:
                if (isLeft) {
                    Points.add(CargoShipMid);
                    Points.add(CargoShip1);
                } else {
                    Points.add(RCargoShipMid);
                    Points.add(RCargoShip1);

                }
                break;
            case 2:
                if (isLeft) {
                    Points.add(CargoShipMid);
                    Points.add(CargoShip2);
                } else {
                    Points.add(RCargoShipMid);
                    Points.add(RCargoShip2);

                }
                break;
        }
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> HabToCargoShip(boolean reversed, boolean isLeft, int bay) {
        List<Pose2d> Points = new ArrayList<>();
        switch (bay) {
            case 1:
                if (isLeft) {
                    Points.add(HabStart);
                    Points.add(CargoShipMid);
                    Points.add(CargoShip1);
                } else {
                    Points.add(HabStart);
                    Points.add(RCargoShipMid);
                    Points.add(RCargoShip1);

                }
                break;
            case 2:
                if (isLeft) {
                    Points.add(HabStart);
                    Points.add(CargoShipMid);
                    Points.add(CargoShip2);
                } else {
                    Points.add(HabStart);
                    Points.add(RCargoShipMid);
                    Points.add(RCargoShip2);

                }
                break;
        }
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60),
                new VelocityLimitRegionConstraint(new Translation2d(30,30), new Translation2d(-72,-120), 48)), 96.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> CargoMidToHairpin(boolean reversed, boolean isLeft) {
        List<Pose2d> Points = new ArrayList<>();
        if (isLeft) {
            Points.add(CargoHoldMid);
            Points.add(HairpinTurn);
        } else {
            Points.add(RCargoHoldMid);
            Points.add(RHairpinTurn);
        }
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> CargoToHairpin(boolean reversed, boolean isLeft) {
        List<Pose2d> Points = new ArrayList<>();
        if (isLeft) {
            Points.add(Cargo);
            Points.add(CargoHoldMid);
            Points.add(HairpinTurn);
        } else {
            Points.add(RCargo);
            Points.add(RCargoHoldMid);
            Points.add(RHairpinTurn);
        }
        return generateTrajectory(reversed, Points, Arrays.asList(
                new CentripetalAccelerationConstraint(60), new VelocityLimitRegionConstraint(new Translation2d(30,30), new Translation2d(-72,-120)
                        , 48)), 96.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> HairpinToCargoShip(boolean reversed, boolean isLeft, int bay) {
        List<Pose2d> Points = new ArrayList<>();
        switch (bay) {
            case 1:
                if (isLeft) {
                    Points.add(HairpinTurn);
                    Points.add(CargoShip1);
                } else {
                    Points.add(RHairpinTurn);
                    Points.add(RCargoShip1);
                }
                break;
            case 2:
                if (isLeft) {
                    Points.add(HairpinTurn);
                    Points.add(CargoShip2);
                } else {
                    Points.add(RHairpinTurn);
                    Points.add(RCargoShip2);
                }
                break;
        }
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 24.0, 60, 10.0);

    }


    public Trajectory<TimedState<Pose2dWithCurvature>> CargoShipToHairpin(boolean reversed, boolean isLeft, int bay) {
        List<Pose2d> Points = new ArrayList<>();
        switch (bay) {
            case 1:
                if (isLeft) {
                    Points.add(CargoShip1);
                    Points.add(HairpinTurn);
                } else {
                    Points.add(RCargoShip1);
                    Points.add(RHairpinTurn);
                }
                break;
            case 2:
                if (isLeft) {
                    Points.add(CargoShip2);
                    Points.add(HairpinTurn);
                } else {
                    Points.add(RCargoShip2);
                    Points.add(RHairpinTurn);
                }
                break;

        }
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> HairpinToCargoMid(boolean reversed, boolean isLeft) {
        List<Pose2d> Points = new ArrayList<>();
        if (isLeft) {
            Points.add(HairpinTurn);
            Points.add(CargoHoldMid);
        } else {
            Points.add(RHairpinTurn);
            Points.add(RCargoHoldMid);
        }
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> CargoMidtoCargo(boolean reversed, boolean isLeft) {
        List<Pose2d> Points = new ArrayList<>();
        if (isLeft) {
            Points.add(CargoHoldMid);
            Points.add(Cargo);
        } else {

            Points.add(RCargoHoldMid);
            Points.add(RCargo);
        }
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 60, 10.0);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> HairpinToCargo(boolean reversed, boolean isLeft) {
        List<Pose2d> Points = new ArrayList<>();
        if (isLeft) {
            Points.add(HairpinTurn);
            Points.add(CargoHoldMid);
            Points.add(Cargo);
        } else {
            Points.add(RHairpinTurn);
            Points.add(RCargoHoldMid);
            Points.add(RCargo);
        }
        return generateTrajectory(reversed, Points, Arrays.asList(new CentripetalAccelerationConstraint(60),new VelocityLimitRegionConstraint(new Translation2d(30,30), new Translation2d(-72,-120), 48)), 96.0, 60, 10.0);
    }
}