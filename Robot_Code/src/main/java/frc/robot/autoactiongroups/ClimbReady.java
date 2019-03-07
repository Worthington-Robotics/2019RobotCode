package frc.robot.autoactiongroups;

import frc.lib.physics.ArmModel;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.ArmStateWaitAction;
import frc.robot.actions.armactions.ArmAction;
import frc.robot.subsystems.Arm;

public class ClimbReady extends StateMachineDescriptor {
    public ClimbReady(){
        addSequential(new ArmAction(Arm.ArmStates.CLIMB_TRANSPORT), 10);
        addSequential(new ArmStateWaitAction(Arm.ArmStates.CLIMB_TRANSPORT, 100), 10000);
        addSequential(new ArmAction(Arm.ArmStates.CLIMB_READY), 10);
    }
}
