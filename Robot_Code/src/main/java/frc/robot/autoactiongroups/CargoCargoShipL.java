package frc.robot.autoactiongroups;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DriveTra;
import frc.robot.actions.waitactions.LineCrossWait;
import frc.robot.actions.waitactions.PointCloudWait;
import frc.robot.planners.DriveTrajectoryGenerator;
import frc.robot.subsystems.Arm;

public class CargoCargoShipL extends StateMachineDescriptor {
    public CargoCargoShipL() {
        Arm.getInstance().setStowed(false);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().HabToOffHab(false), false), 1000);
        addSequential(new LineCrossWait(52, true), 2000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().OffHabToHab(true), false), 1000);
        addSequential(new PointCloudWait(new Pose2d(987, 987, Rotation2d.fromDegrees(0)), 0, 0, 0), 750);

    }
}