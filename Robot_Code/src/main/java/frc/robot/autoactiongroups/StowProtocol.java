package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.ArmAction;
import frc.robot.subsystems.Arm;

public class StowProtocol extends StateMachineDescriptor {
   public StowProtocol()
   {
       addSequential(new ArmAction(Arm.ArmStates.UNSTOW_ARM), 3000);
       addSequential(new ArmAction(Arm.ArmStates.STOW_ARM), 3000);
       Arm.getInstance().setStowed(true);
   }
}
