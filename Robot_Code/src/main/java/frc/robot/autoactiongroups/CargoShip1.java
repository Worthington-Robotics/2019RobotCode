package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.DriveTra;
import frc.robot.actions.PointCloudWait;
import frc.robot.actions.armactions.ArmAction;
import frc.robot.actions.armactions.ManipulatorAction;
import frc.robot.actions.buttonactions.ButtonWaitAction;
import frc.robot.planners.DriveTrajectoryGenerator;
import frc.robot.subsystems.Arm;

public class CargoShip1 extends StateMachineDescriptor {
    public CargoShip1(boolean isLeft)
    {
        Arm.getInstance().setStowed(false);
        /* addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().HabToOffHab(false, false), false), new ButtonWaitAction(new Joystick(2), 1)}, 10000);
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().OffHabToHab(false, false), false), new ButtonWaitAction(new Joystick(2), 1)}, 10000);
*/
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().HabToCargoShip(false, isLeft, 1)), 1);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().LCargoShipMid , 10,100000,180), 10000);
        addSequential(new ArmAction(Arm.ArmStates.CARGO_SHIP_CARGO), 1);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().LCargoShip1,20,10,360), 10000);
        addSequential(new ManipulatorAction(ManipulatorAction.ShotPower.SlowShoot),700);
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().CargoShipToHairpin(true, isLeft, 1))} , 21);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().LHairpinTurn,10,10,45) , 10000);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().HairpinToCargo(false, isLeft)), new ArmAction(Arm.ArmStates.FWD_GROUND_CARGO)}, 21);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().LCargo,10,10,45), 10000);
        addSequential(new ManipulatorAction(ManipulatorAction.ShotPower.PickUp), 1000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().CargoToHairpin(true, isLeft)), 21);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().LHairpinTurn,10,10,45), 10000);
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().HairpinToCargoShip(false, isLeft, 2)), new ArmAction(Arm.ArmStates.CARGO_SHIP_CARGO)}, 21);
        addSequential(new PointCloudWait(DriveTrajectoryGenerator.getInstance().LCargoShip2,10,10,45), 10000);
        addSequential(new ManipulatorAction(ManipulatorAction.ShotPower.SlowShoot), 600);
    }
}
