package frc.robot.autoactiongroups;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.DriveTra;
import frc.robot.planners.TraGenerator;
import frc.robot.subsystems.Drive;

public class FrontOfCargo extends StateMachineDescriptor {
    public FrontOfCargo() {
        addSequential(new DriveTra(TraGenerator.getInstance().getTenFeet(), false), 30000 );


    }
}