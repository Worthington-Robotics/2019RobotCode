package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.armactions.ArmAction;
import frc.robot.actions.armactions.ManipulatorAction;
import frc.robot.actions.armactions.UnstowArmAction;
import frc.robot.actions.buttonactions.ButtonWaitAction;
import frc.robot.actions.driveactions.DriveTra;
import frc.robot.planners.DriveTrajectoryGenerator;
import frc.robot.subsystems.Arm;


public class AutoTestProtocol extends StateMachineDescriptor {
    public AutoTestProtocol(){
        addParallel(new Action[]{new ManipulatorAction(ManipulatorAction.ShotPower.Shoot),
                new DriveTra(DriveTrajectoryGenerator.getInstance().getTenFeet(), false),
                new UnstowArmAction()},3000);
        addSequential(new ButtonWaitAction(Constants.MASTER, 11), 2000);
        addParallel(new Action[]{new ManipulatorAction(ManipulatorAction.ShotPower.PickUp),
                new ArmAction(Arm.ArmStates.FWD_GROUND_CARGO)},3000);
        addSequential(new ButtonWaitAction(Constants.MASTER, 11), 2000);
        addSequential(new ButtonWaitAction(Constants.MASTER, 11), 2000);
        addSequential(new ArmAction(Arm.ArmStates.FWD_LOW_CARGO), 3000);
        addSequential(new ButtonWaitAction(Constants.MASTER, 11), 2000);
        addSequential(new ButtonWaitAction(Constants.MASTER, 11), 2000);
        addSequential(new ArmAction(Arm.ArmStates.FWD_MEDIUM_CARGO), 3000);
        addSequential(new ButtonWaitAction(Constants.MASTER, 11), 2000);
        addSequential(new ButtonWaitAction(Constants.MASTER, 11), 2000);
        addSequential(new ArmAction(Arm.ArmStates.FWD_HIGH_CARGO), 3000);
        addSequential(new ButtonWaitAction(Constants.MASTER, 11), 2000);
        addSequential(new ArmAction(Arm.ArmStates.UNSTOW_ARM), 3000);
        addSequential(new ButtonWaitAction(Constants.MASTER, 11), 2000);
        addSequential(new ArmAction(Arm.ArmStates.STOW_ARM), 2000);
    }
}
