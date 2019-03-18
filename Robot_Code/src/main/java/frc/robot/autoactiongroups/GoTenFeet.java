package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.DriveTra;
import frc.robot.actions.armactions.ArmAction;
import frc.robot.actions.armactions.ManipulatorAction;
import frc.robot.actions.armactions.UnstowArmAction;
import frc.robot.planners.DriveTrajectoryGenerator;
import frc.robot.subsystems.Arm;

public class GoTenFeet extends StateMachineDescriptor {
    public GoTenFeet() {
        addParallel(new Action[]{new DriveTra(DriveTrajectoryGenerator.getInstance().getTenFeet(), false) } , 3000);


    }
}
