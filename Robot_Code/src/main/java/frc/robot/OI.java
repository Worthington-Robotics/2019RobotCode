package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.actions.*;
import frc.lib.statemachine.Action;

public class OI{

    public OI(){
        Button second2 = new JoystickButton(Constants.SECOND, 2);
        Button second3 = new JoystickButton(Constants.SECOND, 3);
        second2.whileHeld(Action.toCommand(new ManipulatorAction(ManipulatorAction.ShotPower.PickUp)));
        second3.whileHeld(Action.toCommand(new ManipulatorAction(ManipulatorAction.ShotPower.Shoot)));
        Button exit = new JoystickButton(Constants.MASTER, 1);
        exit.whenPressed(Action.toCommand(new StopMachineAction()));
    }

}