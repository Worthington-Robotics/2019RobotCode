package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.DriveTra;
import frc.robot.actions.armactions.ManipulatorAction;
import frc.robot.planners.DriveTrajectoryGenerator;

public class GoTenFeet extends StateMachineDescriptor {
    public GoTenFeet() {

        addParallel(new Action[]{ new ManipulatorAction(ManipulatorAction.ShotPower.Shoot), new DriveTra(DriveTrajectoryGenerator.getInstance().getTenFeet(), false) } , 3000);


    }
}
