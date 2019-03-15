package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.DriveTra;
import frc.robot.actions.PointCloudWait;
import frc.robot.actions.armactions.ArmAction;
import frc.robot.actions.armactions.ManipulatorAction;
import frc.robot.planners.DriveTrajectoryGenerator;
import frc.robot.subsystems.Arm;

public class CargoShip2 extends StateMachineDescriptor {
    public CargoShip2() {
        Arm.getInstance().setStowed(false);
    addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().HabToOffHab(false, false), false)}, 10000);
    addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().OffHabToHab(false, false), false)}, 10000);
    addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().HabToCargoShip(false, false, 1), true), 10);
/*
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().HabToCargoShip(false, true, 1)), 1);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().CargoShipMid, 10, 100000, 180), 10000);
        addSequential(new ArmAction(Arm.ArmStates.CARGO_SHIP_CARGO), 1);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().CargoShip1, 20, 10, 360), 10000);
        addSequential(new ManipulatorAction(ManipulatorAction.ShotPower.SlowShoot), 700);
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().CargoShipToHairpin(true, true, 1))}, 21);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().HairpinTurn, 10, 10, 45), 10000);
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().HairpinToCargo(false, true)), new ArmAction(Arm.ArmStates.FWD_GROUND_CARGO)}, 21);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().Cargo, 10, 10, 45), 10000);
        addSequential(new ManipulatorAction(ManipulatorAction.ShotPower.PickUp), 1000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().CargoToHairpin(true, true)), 21);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().HairpinTurn, 10, 10, 45), 10000);
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().HairpinToCargoShip(false, true, 2)), new ArmAction(Arm.ArmStates.CARGO_SHIP_CARGO)}, 21);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().CargoShip2, 10, 10, 45), 10000);
        addSequential(new ManipulatorAction(ManipulatorAction.ShotPower.SlowShoot), 600);
*/
    }
}