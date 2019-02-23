package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.*;
import frc.robot.planners.TraGenerator;


public class AutoTestProtocall extends StateMachineDescriptor {
    public AutoTestProtocall(){
        addParallel(new Action[]{new ManipulatorAction(ManipulatorAction.ShotPower.Shoot), new AlienAction(), new DriveTra(TraGenerator.getInstance().autoTestProcedure(), false)},3000);
        addParallel(new Action[]{new ManipulatorAction(ManipulatorAction.ShotPower.PickUp), new AlienAction(), new VisionTra(), new AnglePID()},3000);
    }
}
