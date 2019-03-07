package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.DriveTra;
import frc.robot.actions.armactions.AlienAction;
import frc.robot.actions.armactions.ArmAction;
import frc.robot.planners.DriveTrajectoryGenerator;
import frc.robot.subsystems.Arm;
/*
public class LeftBackHatchesHighAndMid extends StateMachineDescriptor {
    public LeftBackHatchesHighAndMid() {
        long timems = 10000;
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().hatchLeftHighAndMid1()), timems);
        addParallel(new Action[] {new ArmAction(Arm.ArmStates.FWD_HIGH_HATCH), new DriveTra(DriveTrajectoryGenerator.getInstance().hatchLeftHighAndMid2())}, timems);
        addSequential(new AlienAction(), timems);
        addParallel(new Action[] {new DriveTra(DriveTrajectoryGenerator.getInstance().hatchLeftHighAndMid3()), new ArmAction(Arm.ArmStates.UNSTOW_ARM)}, timems);
    }
}
*/