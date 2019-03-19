package frc.robot.autoactiongroups;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.waitactions.ArmStateWaitAction;
import frc.robot.actions.armactions.ArmAction;
import frc.robot.actions.armactions.StowArmAction;
import frc.robot.subsystems.Arm;

public class StowProtocol extends StateMachineDescriptor {
   public StowProtocol()
   {
       addSequential(new ArmAction(Arm.ArmStates.UNSTOW_ARM), 1000);
       addSequential(new ArmStateWaitAction(Arm.ArmStates.UNSTOW_ARM, 100), 3000);
       addSequential(new StowArmAction(), 1000);
   }
}
