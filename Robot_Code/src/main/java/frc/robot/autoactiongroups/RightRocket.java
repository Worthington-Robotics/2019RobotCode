package frc.robot.autoactiongroups;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.DriveTra;
import frc.robot.planners.TraGenerator;

public class RightRocket extends StateMachineDescriptor {
    public RightRocket() {
        addSequential(new DriveTra(TraGenerator.getInstance().getTenFeet(), false), );
    }
}
