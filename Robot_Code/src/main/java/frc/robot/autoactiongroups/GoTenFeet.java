package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.DriveTra;
import frc.robot.actions.ManipulatorAction;
import frc.robot.planners.TraGenerator;

public class GoTenFeet extends StateMachineDescriptor {
    public GoTenFeet() {

        addParallel(new Action[]{ new ManipulatorAction(ManipulatorAction.ShotPower.Shoot), new DriveTra(TraGenerator.getInstance().getTenFeet(), false) } , 3000);


    }
}
