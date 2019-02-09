package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.lib.statemachine.Action;
import frc.robot.actions.AStopAction;
import frc.robot.actions.AlienAction;
import frc.robot.actions.AnglePID;
import frc.robot.actions.ManipulatorAction;

public class OI{

    public OI(){
        Button second1 = new JoystickButton(Constants.LAUNCH_PAD, 1);
        Button second2 = new JoystickButton(Constants.LAUNCH_PAD, 2);
        Button second3 = new JoystickButton(Constants.LAUNCH_PAD, 3);
        Button second4 = new JoystickButton(Constants.LAUNCH_PAD, 4);
        Button second5 = new JoystickButton(Constants.LAUNCH_PAD, 5);
        Button second6 = new JoystickButton(Constants.LAUNCH_PAD, 6);
        Button second7 = new JoystickButton(Constants.LAUNCH_PAD, 7);
        Button second8 = new JoystickButton(Constants.LAUNCH_PAD, 8);
        Button second9 = new JoystickButton(Constants.LAUNCH_PAD, 9);
        Button second10 = new JoystickButton(Constants.LAUNCH_PAD, 10);
        Button second11 = new JoystickButton(Constants.LAUNCH_PAD, 11);
        Button second12 = new JoystickButton(Constants.LAUNCH_PAD, 12);
        Button second13 = new JoystickButton(Constants.LAUNCH_PAD, 13);

        second1.whileHeld(Action.toCommand(new AStopAction()));
        second2.whileHeld(Action.toCommand(new ManipulatorAction(ManipulatorAction.ShotPower.PickUp)));
        second3.whileHeld(Action.toCommand(new ManipulatorAction(ManipulatorAction.ShotPower.Shoot)));
        second4.whileHeld(Action.toCommand(new AlienAction()));
        second5.whileHeld(Action.toCommand(new AlienAction()));
        second6.whileHeld(Action.toCommand(new AlienAction()));
        second7.whileHeld(Action.toCommand(new AlienAction()));
        second8.whileHeld(Action.toCommand(new AlienAction()));
        second9.whileHeld(Action.toCommand(new AlienAction()));
        second10.whileHeld(Action.toCommand(new AlienAction()));
        second11.whileHeld(Action.toCommand(new AlienAction()));
        second12.whileHeld(Action.toCommand(new AlienAction()));
        second13.whileHeld(Action.toCommand(new AlienAction()));

        Button anglePidButton = new JoystickButton(Constants.MASTER, 7);
        anglePidButton.whileHeld(Action.toCommand(new AnglePID()));
    }

}