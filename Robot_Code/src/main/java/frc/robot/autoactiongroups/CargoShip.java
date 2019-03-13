package frc.robot.autoactiongroups;

import edu.wpi.first.wpilibj.Joystick;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.DriveTra;
import frc.robot.actions.PointCloudWait;
import frc.robot.actions.armactions.ArmAction;
import frc.robot.actions.armactions.ManipulatorAction;
import frc.robot.actions.buttonactions.ButtonWaitAction;
import frc.robot.planners.DriveTrajectoryGenerator;
import frc.robot.subsystems.Arm;

public class CargoShip extends StateMachineDescriptor {
    public CargoShip(boolean isLeft) {
        Arm.getInstance().setStowed(false);
       /* addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().HabToOffHab(false, false), false), new ButtonWaitAction(new Joystick(2), 1)}, 10000);
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().OffHabToHab(false, false), false), new ButtonWaitAction(new Joystick(2), 1)}, 10000);
*/
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().HabToCargoShipMid(false, isLeft), false),  new PointCloudWait(DriveTrajectoryGenerator.getInstance().CargoShipMid, 2,2,2)}, 5000);
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().CargoShipMidToCargoShip(false, isLeft, 1), false),/*new ArmAction(Arm.ArmStates.CARGO_SHIP_CARGO),*/ new PointCloudWait(DriveTrajectoryGenerator.getInstance().CargoShip1, 2,2,2)}, 5000);
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().CargoShipToHairpin(true, isLeft, 1), false), /*new ManipulatorAction(ManipulatorAction.ShotPower.Shoot),*/ new PointCloudWait(DriveTrajectoryGenerator.getInstance().HairpinTurn ,2,2,2)}, 5000);
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().HairpinToCargoMid(false, isLeft), false), /*new ArmAction(Arm.ArmStates.FWD_GROUND_CARGO),*/ new PointCloudWait(DriveTrajectoryGenerator.getInstance().CargoHoldMid,2,2,2)}, 5000);
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().CargoMidtoCargo(false, isLeft), false), /*new ManipulatorAction(ManipulatorAction.ShotPower.PickUp),*/ new PointCloudWait(DriveTrajectoryGenerator.getInstance().Cargo,2,2,2)}, 6000);
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().CargoToCargoMid(true, isLeft), false), new PointCloudWait(DriveTrajectoryGenerator.getInstance().CargoHoldMid,2,2,2)}, 6000);
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().CargoMidToHairpin(true, isLeft), false), /*new ArmAction(Arm.ArmStates.CARGO_SHIP_CARGO),*/ new PointCloudWait(DriveTrajectoryGenerator.getInstance().HairpinTurn,2,2,2)}, 5000);
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().HairpinToCargoShip(false, isLeft, 2), false), new PointCloudWait(DriveTrajectoryGenerator.getInstance().CargoShip2,2,2,2)}, 5000);
        addSequential(new ManipulatorAction(ManipulatorAction.ShotPower.Shoot), 10);
    }

}

