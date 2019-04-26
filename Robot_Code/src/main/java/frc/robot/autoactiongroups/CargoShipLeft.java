package frc.robot.autoactiongroups;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.armactions.ArmSoftCal;
import frc.robot.actions.armactions.ManipulatorAction;
import frc.robot.actions.armactions.PistonArmAction;
import frc.robot.actions.buttonactions.ButtonWaitAction;
import frc.robot.actions.driveactions.DriveTra;
import frc.robot.actions.waitactions.LineCrossWait;
import frc.robot.actions.waitactions.PointCloudWait;
import frc.robot.planners.DriveTrajectoryGenerator;
import frc.robot.subsystems.Arm;

public class CargoShipLeft extends StateMachineDescriptor {
    public CargoShipLeft(boolean doReset) {
        if(doReset) addSequential(new ArmSoftCal(), 21);
        Arm.getInstance().setIgnoreSafety(true);
        Arm.getInstance().setStowed(false);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().HabToOffHab(false), false), 1000);
        addSequential(new LineCrossWait(52, true), 2000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().OffHabToHab(true), false), 1000);
        addSequential(new PointCloudWait(new Pose2d(987, 987, Rotation2d.fromDegrees(0)), 0, 0, 0), 1000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().HabToCargoShip(false, true, 1), true), 1000);
        addSequential(new LineCrossWait(72, true), 10000);
        addSequential(new PistonArmAction(Arm.PistonArmStates.A_CARGO_SHIP_CARGO), 20);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().CargoShip1, 20, 4, 360), 12000);
        addSequential(new ManipulatorAction(ManipulatorAction.ShotPower.Shoot), 1000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().CargoShipToHairpin(true, true, 1)), 21);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().HairpinTurn, 30, 30, 45), 10000);
        addSequential(new PistonArmAction(Arm.PistonArmStates.FWD_GROUND_CARGO), 20);
        addSequential(new ButtonWaitAction(Constants.MASTER, 14), 250);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().HairpinToCargo(false, true)), 21);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().Cargo, 20, 20, 180), 5000);
        addSequential(new ManipulatorAction(ManipulatorAction.ShotPower.PickUp), 1500);

        /*
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().CargoToHairpin(true, false)), 21);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().RHairpinTurn, 20, 200, 45), 10000);
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().HairpinToCargoShip(false, false, 2)), new ArmAction(Arm.ArmStates.CARGO_SHIP_CARGO)}, 21);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().RCargoShip2, 10, 10, 45), 10000);
        addSequential(new ManipulatorAction(ManipulatorAction.ShotPower.SlowShoot), 600);*/
    }
}