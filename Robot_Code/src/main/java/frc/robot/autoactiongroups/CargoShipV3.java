package frc.robot.autoactiongroups;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.armactions.ManipulatorAction;
import frc.robot.actions.driveactions.DriveTra;
import frc.robot.actions.waitactions.LineCrossWait;
import frc.robot.planners.DriveTrajectoryGenerator;
import frc.robot.subsystems.Arm;

public class CargoShipV3 extends StateMachineDescriptor {
    public CargoShipV3(boolean isLeft) {
        Arm.getInstance().setStowed(false);
        /*
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().HabToOffHab(false), false), 1000);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().HabOff,1,100,180), 3000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().OffHabToHab(true), false), 1000);
        addSequential(new PointCloudWait(new Pose2d(987,987, Rotation2d.fromDegrees(0)),0,0,0), 1000);
        */
        if (isLeft)
            addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().HabToCargoShip(true, true, 1)), 21);
        addSequential(new LineCrossWait(-3, false), 10000);
        addSequential(new ManipulatorAction(ManipulatorAction.ShotPower.SlowShoot), 500);

    }
}
