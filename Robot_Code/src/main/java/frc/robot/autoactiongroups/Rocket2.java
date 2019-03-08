package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.DriveTra;
import frc.robot.actions.armactions.ArmAction;
import frc.robot.actions.armactions.UnstowArmAction;
import frc.robot.actions.buttonactions.ButtonWaitAction;
import frc.robot.planners.DriveTrajectoryGenerator;
import frc.robot.subsystems.Arm;

public class Rocket2 extends StateMachineDescriptor {
    public Rocket2()
    {
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().RevHabToRocketMid(), false), 10);
        addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().RevRocketMidToRocketApproch(), false), new UnstowArmAction()}, 10);
        addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().RocketApprochToRocket()), new ArmAction(Arm.ArmStates.FWD_MEDIUM_HATCH)}, 10);
        addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().RevRocketToRocketApproch()), new ArmAction(Arm.ArmStates.FWD_GROUND_CARGO)}, 10);
        addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().RocketApprochToRocketMidpoint(), false), 10);
        addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().RocketMidPointToHatchPickup()), new ArmAction(Arm.ArmStates.FWD_LOW_HATCH)}, 10);
        addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().RevHatchPickupToRocketMidPoint(), false), 10);
        addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().RevRocketMidToRocketApproch(), false), 10);
        addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().RocketApprochToRocket()), new ArmAction(Arm.ArmStates.FWD_HIGH_HATCH)}, 10);

    }
}
