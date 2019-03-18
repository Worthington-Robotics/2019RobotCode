package frc.robot.autoactiongroups;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.DriveTra;
import frc.robot.actions.LineCrossWait;
import frc.robot.actions.PointCloudWait;
import frc.robot.actions.armactions.ArmAction;
import frc.robot.actions.armactions.ManipulatorAction;
import frc.robot.actions.buttonactions.ButtonWaitAction;
import frc.robot.planners.DriveTrajectoryGenerator;
import frc.robot.subsystems.Arm;

public class CargoShip2 extends StateMachineDescriptor {
    public CargoShip2() {
        Arm.getInstance().setStowed(false);
        /*
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().HabToOffHab(false), false), 1000);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().HabOff,1,100,180), 3000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().OffHabToHab(true), false), 1000);
        addSequential(new PointCloudWait(new Pose2d(987,987, Rotation2d.fromDegrees(0)),0,0,0), 1000);
        */
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().HabToCargoShip(false, true, 1), true), 1000);
        addSequential(new LineCrossWait(131, true), 10000);
        addSequential(new ArmAction(Arm.ArmStates.CARGO_SHIP_CARGO), 1);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().CargoShip1, 20, 3, 360), 10000);
        addSequential(new ManipulatorAction(ManipulatorAction.ShotPower.Shoot), 700);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().CargoShipToHairpin(true, true, 1)), 21);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().HairpinTurn, 30, 30, 45), 10000);
        addSequential(new  ArmAction(Arm.ArmStates.FWD_GROUND_CARGO), 20);
        addSequential(new ButtonWaitAction(Constants.MASTER, 14), 1000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().HairpinToCargo(false, true)), 21);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().Cargo, 10, 10, 180), 10000);
        addSequential(new ManipulatorAction(ManipulatorAction.ShotPower.PickUp), 1500);
        /*
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().CargoToHairpin(true, true)), 21);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().HairpinTurn, 20, 200, 45), 10000);
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().HairpinToCargoShip(false, true, 2)), new ArmAction(Arm.ArmStates.CARGO_SHIP_CARGO)}, 21);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().CargoShip2, 10, 10, 45), 10000);
        addSequential(new ManipulatorAction(ManipulatorAction.ShotPower.SlowShoot), 600);
   */
    }
}