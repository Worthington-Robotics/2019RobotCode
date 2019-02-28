package frc.robot.autoactiongroups;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.DriveTra;
import frc.robot.planners.DriveTrajectoryGenerator;

public class FrontOfCargo extends StateMachineDescriptor {
    public FrontOfCargo() {
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().getTenFeet(), false), 30000 );


    }
}