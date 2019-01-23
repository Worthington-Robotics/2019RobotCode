package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.actions.*;
import frc.lib.statemachine.Action;

public class OI{

    public OI(){

        Button second2 = new JoystickButton(Constants.SECOND, 2);
        Button second3 = new JoystickButton(Constants.SECOND, 3);
        Button second4 = new JoystickButton(Constants.SECOND, 4);
        Button second5 = new JoystickButton(Constants.SECOND, 5);
        second2.whileHeld(Action.toCommand(new ManipulatorAction(ManipulatorAction.ShotPower.PickUp)));
        second3.whileHeld(Action.toCommand(new ManipulatorAction(ManipulatorAction.ShotPower.Shoot)));
        second4.whileHeld(Action.toCommand(new AlienAction()));
        second5.whileHeld(Action.toCommand(new AlienAction()));

    }

}