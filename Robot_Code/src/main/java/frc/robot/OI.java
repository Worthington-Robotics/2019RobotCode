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
        Button Launchpad1 = new JoystickButton(Constants.LAUNCH_PAD, 1);
        Button Launchpad2 = new JoystickButton(Constants.LAUNCH_PAD, 2);
        Button Launchpad3 = new JoystickButton(Constants.LAUNCH_PAD, 3);
        Button Launchpad4 = new JoystickButton(Constants.LAUNCH_PAD, 4);
        Button Launchpad5 = new JoystickButton(Constants.LAUNCH_PAD, 5);
        Button Launchpad6 = new JoystickButton(Constants.LAUNCH_PAD, 6);
        Button Launchpad7 = new JoystickButton(Constants.LAUNCH_PAD, 7);
        Button Launchpad8 = new JoystickButton(Constants.LAUNCH_PAD, 8);
        Button Launchpad9 = new JoystickButton(Constants.LAUNCH_PAD, 9);
        Button Launchpad10 = new JoystickButton(Constants.LAUNCH_PAD, 10);
        Button Launchpad11 = new JoystickButton(Constants.LAUNCH_PAD, 11);
        Button Launchpad12 = new JoystickButton(Constants.LAUNCH_PAD, 12);
        Button Launchpad13 = new JoystickButton(Constants.LAUNCH_PAD, 13);
        Button Launchpad14 = new JoystickButton(Constants.LAUNCH_PAD,14);
        Button Launchpad15 = new JoystickButton(Constants.LAUNCH_PAD,15);
        Button Launchpad16 = new JoystickButton(Constants.LAUNCH_PAD,16);


        /*Launchpad1.whileHeld(Action.toCommand(new ));
        Launchpad2.whileHeld(Action.toCommand(new ));
        Launchpad3.whileHeld(Action.toCommand(new ));
        Launchpad4.whileHeld(Action.toCommand(new ));
        Launchpad5.whileHeld(Action.toCommand(new ));
        Launchpad6.whileHeld(Action.toCommand(new ));
        Launchpad7.whileHeld(Action.toCommand(new ));
        Launchpad8.whileHeld(Action.toCommand(new ));
        Launchpad9.whileHeld(Action.toCommand(new ));
        Launchpad10.whileHeld(Action.toCommand(new ));
        Launchpad11.whileHeld(Action.toCommand(new ));
        Launchpad12.whileHeld(Action.toCommand(new ));
        Launchpad13.whileHeld(Action.toCommand(new ));*/

        Button anglePidButton = new JoystickButton(Constants.MASTER, 7);
        anglePidButton.whileHeld(Action.toCommand(new AnglePID()));
    }

}