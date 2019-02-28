package frc.robot.autoactiongroups;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.DriveTra;
import frc.robot.planners.DriveTrajectoryGenerator;

public class LeftRocket extends StateMachineDescriptor{
    public LeftRocket() {
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().getRightRocket(), false), 300000);
    }
}
