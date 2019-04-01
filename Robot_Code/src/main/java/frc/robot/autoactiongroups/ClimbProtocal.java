package frc.robot.autoactiongroups;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.Elevator;
import frc.robot.actions.armactions.ArmAction;
import frc.robot.actions.waitactions.ArmStateWaitAction;
import frc.robot.subsystems.Arm;

public class ClimbProtocal extends StateMachineDescriptor {
    public ClimbProtocal()
    {
        addSequential(new ArmAction(Arm.ArmStates.FWD_MEDIUM_CARGO), 20);
        addSequential(new ArmAction(Arm.ArmStates.FWD_GROUND_CARGO) ,20);
        addSequential(new ArmStateWaitAction(Arm.ArmStates.CLIMB_MID_CHECK, 100), 2000);
        addSequential(new Elevator(-.5), 3500);
    }
}
