package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.actions.*;
import frc.lib.statemachine.Action;

public class OI{

    public OI(){
        Button Launch1 = new JoystickButton(Constants.LAUNCH_PAD, 1);
        Button second2 = new JoystickButton(Constants.LAUNCH_PAD, 2);
        Button second3 = new JoystickButton(Constants.LAUNCH_PAD, 3);
        Button second4 = new JoystickButton(Constants.LAUNCH_PAD, 4);
        Button second5 = new JoystickButton(Constants.LAUNCH_PAD, 5);
        Launch1.whileHeld(Action.toCommand(new AStopAction()));
        second2.whileHeld(Action.toCommand(new ManipulatorAction(ManipulatorAction.ShotPower.PickUp)));
        second3.whileHeld(Action.toCommand(new ManipulatorAction(ManipulatorAction.ShotPower.Shoot)));
        second4.whileHeld(Action.toCommand(new AlienAction()));
        second5.whileHeld(Action.toCommand(new AlienAction()));

        Button anglePidButton = new JoystickButton(Constants.MASTER, 7);
        anglePidButton.whenPressed(Action.toCommand(new AnglePID()));
    }

}