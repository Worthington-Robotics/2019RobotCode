package frc.robot.autoactiongroups;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.DriveTra;
import frc.robot.planners.TraGenerator;

public class FrontOfCargo extends StateMachineDescriptor {
    public FrontOfCargo() {

        addSequential(new DriveTra(TraGenerator.getInstance().getTenFeet(), false), 30000 );


    }
}