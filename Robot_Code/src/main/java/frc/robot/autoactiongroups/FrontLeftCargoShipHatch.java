package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.ArmAction;
import frc.robot.actions.DriveTra;
import frc.robot.planners.DriveTrajectoryGenerator;
import frc.robot.subsystems.Arm;

public class FrontLeftCargoShipHatch extends StateMachineDescriptor {
    public FrontLeftCargoShipHatch() {
        addParallel(new Action[]{new ArmAction(Arm.ArmStates.FWD_LOW_HATCH), new DriveTra(DriveTrajectoryGenerator.getInstance().frontLeftCargoShipHatch())},3000);
    }
}
