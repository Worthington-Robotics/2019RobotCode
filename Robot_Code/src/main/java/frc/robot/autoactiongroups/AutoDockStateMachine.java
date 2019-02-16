package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.*;
import frc.robot.planners.TraGenerator;
public class AutoDockStateMachine extends StateMachineDescriptor{
    public AutoDockStateMachine()
    {
        addSequential(new AnglePID() , 3000);
        addSequential(new VisionTra() , 3000);
        addParallel(new Action[]{new AlienAction(), new ManipulatorAction(ManipulatorAction.ShotPower.Shoot)}, 300);
    }
}
