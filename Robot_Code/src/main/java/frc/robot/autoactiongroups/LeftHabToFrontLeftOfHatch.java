package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.ArmStateWaitAction;
import frc.robot.actions.DriveTra;
import frc.robot.actions.armactions.AlienAction;
import frc.robot.actions.armactions.ArmAction;
import frc.robot.actions.armactions.ManipulatorAction;
import frc.robot.actions.armactions.UnstowArmAction;
import frc.robot.planners.DriveTrajectoryGenerator;
import frc.robot.subsystems.Arm;

public class LeftHabToFrontLeftOfHatch extends StateMachineDescriptor {
    public LeftHabToFrontLeftOfHatch() {
        addSequential(new UnstowArmAction(), 10000);
        addSequential(new ArmStateWaitAction(Arm.ArmStates.UNSTOW_ARM, 100), 10);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().LeftHabToFrontLeftOfHatch1()), new ArmAction(Arm.ArmStates.FWD_LOW_HATCH)}, 10000);
        addSequential(new AlienAction(), 10000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().LeftHabToFrontLeftOfHatch2()), 10000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().LeftHabToFrontLeftOfHatch3()), 10000);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().LeftHabToFrontLeftOfHatch4()), new ArmAction(Arm.ArmStates.FWD_GROUND_CARGO)}, 10000);
        addSequential(new ManipulatorAction(ManipulatorAction.ShotPower.PickUp),10000);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().LeftHabToFrontLeftOfHatch5()), new ArmAction(Arm.ArmStates.FWD_MEDIUM_CARGO)}, 10000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().LeftHabToFrontLeftOfHatch6()), 10000);
        addSequential(new ManipulatorAction(ManipulatorAction.ShotPower.Shoot), 10000);
    }
}