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

public class LRocket {
    public class Rocket2 extends StateMachineDescriptor {
        public Rocket2()
        {
            addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().RevHabToLRocketMid(), false), 10);
            addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
            addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().RevLRocketMidToLRocketApproch(), false), new UnstowArmAction()}, 10);
            addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
            addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().LRocketApprochToLRocket()), new ArmAction(Arm.ArmStates.FWD_MEDIUM_HATCH)}, 10);
            addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
            addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().RevLRocketToLRocketApproch()), new ArmAction(Arm.ArmStates.FWD_GROUND_CARGO)}, 10);
            addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
            addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().LRocketApprochToLRocketMidpoint(), false), 10);
            addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
            addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().LRocketMidPointToLHatchPickup()), new ArmAction(Arm.ArmStates.FWD_LOW_HATCH)}, 10);
            addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
            addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().RevLHatchPickupToLRocketMidPoint(), false), 10);
            addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
            addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().RevLRocketMidToLRocketApproch(), false), 10);
            addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
            addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().LRocketApprochToLRocket()), new ArmAction(Arm.ArmStates.FWD_HIGH_HATCH)}, 10);

        }
    }
}
