package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.actions.*;
import frc.lib.statemachine.Action;

public class OI{

    public OI(){
        Button exit = new JoystickButton(Constants.MASTER, 1);
        exit.whenPressed(Action.toCommand(new StopMachineAction()));
    }

}