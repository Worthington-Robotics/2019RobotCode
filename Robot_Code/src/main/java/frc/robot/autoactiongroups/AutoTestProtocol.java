package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.armactions.ManipulatorAction;
import frc.robot.actions.armactions.PistonArmAction;
import frc.robot.actions.armactions.StowArmAction;
import frc.robot.actions.armactions.UnstowArmAction;
import frc.robot.actions.buttonactions.ButtonWaitAction;
import frc.robot.actions.driveactions.DriveTra;
import frc.robot.planners.DriveTrajectoryGenerator;
import frc.robot.subsystems.Arm;


public class AutoTestProtocol extends StateMachineDescriptor {
    public AutoTestProtocol() {
        addParallel(new Action[]{new ManipulatorAction(ManipulatorAction.ShotPower.Shoot),
                new DriveTra(DriveTrajectoryGenerator.getInstance().getTenInch(), false),
                new UnstowArmAction()}, 2000);
        addSequential(new ButtonWaitAction(Constants.MASTER, 11), 4000);
        addParallel(new Action[]{new ManipulatorAction(ManipulatorAction.ShotPower.PickUp),
                new PistonArmAction(Arm.PistonArmStates.FWD_GROUND_CARGO)}, 2000);
        addSequential(new ButtonWaitAction(Constants.MASTER, 11), 4000);
        addSequential(new PistonArmAction(Arm.PistonArmStates.FWD_LOW_CARGO), 20);
        addSequential(new ButtonWaitAction(Constants.MASTER, 11), 4000);
        addSequential(new PistonArmAction(Arm.PistonArmStates.FWD_MEDIUM_CARGO), 20);
        addSequential(new ButtonWaitAction(Constants.MASTER, 11), 4000);
        addSequential(new PistonArmAction(Arm.PistonArmStates.CARGO_SHIP_CARGO), 20);
        addSequential(new ButtonWaitAction(Constants.MASTER, 11), 4000);
        addSequential(new StowArmAction(), 3000);
    }
}