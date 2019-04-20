package frc.robot.autoactiongroups;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.armactions.PistonArmAction;
import frc.robot.actions.armactions.StowArmAction;
import frc.robot.actions.waitactions.PistonArmStateWait;
import frc.robot.subsystems.Arm;

public class StowProtocol extends StateMachineDescriptor {
    public StowProtocol() {
        addSequential(new PistonArmAction(Arm.PistonArmStates.UNSTOW_ARM), 1000);
        addSequential(new PistonArmStateWait(Arm.PistonArmStates.UNSTOW_ARM, 200), 3000);
        addSequential(new StowArmAction(), 1000);
    }
}
