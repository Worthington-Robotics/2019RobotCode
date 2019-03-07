package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.DriveTra;
import frc.robot.actions.buttonactions.ButtonWaitAction;
import frc.robot.planners.DriveTrajectoryGenerator;

public class Rocket extends StateMachineDescriptor {
    public Rocket()
    {
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().RevHabToRocketMid(), false), 10);
        addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().RevRocketMidToRocketApproch(), false), 10);
        addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().RocketApprochToRocket(), false), 10);
        addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().RevRocketToRocketApproch(), false), 10);
        addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().RocketApprochToRocketMidpoint(), false), 10);
        addSequential(new ButtonWaitAction(Constants.MASTER,4), 30000);
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().RocketMidPointToHatchPickup(), false), 10);
    }
}
