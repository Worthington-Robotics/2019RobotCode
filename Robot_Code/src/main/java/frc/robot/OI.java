package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.lib.statemachine.Action;
import frc.robot.actions.*;

public class OI{

    public OI(){
        Button GroundHatch = new JoystickButton(Constants.LAUNCH_PAD, Constants.GroundHatch);
        Button GroundCargo = new JoystickButton(Constants.LAUNCH_PAD, Constants.GroundCargo);
        Button BotHatch = new JoystickButton(Constants.LAUNCH_PAD, Constants.BottomHatch);
        Button BotCargo = new JoystickButton(Constants.LAUNCH_PAD, Constants.BottomHatch);
        Button MidHatch = new JoystickButton(Constants.LAUNCH_PAD, Constants.MidHatch);
        Button MidCargo = new JoystickButton(Constants.LAUNCH_PAD, Constants.MidCargo);
        Button TopHatch = new JoystickButton(Constants.LAUNCH_PAD, Constants.TopHatch);
        Button TopCargo = new JoystickButton(Constants.LAUNCH_PAD, Constants.TopCargo);
        Button Launchpad10 = new JoystickButton(Constants.LAUNCH_PAD, 10);
        Button Launchpad11 = new JoystickButton(Constants.LAUNCH_PAD, 11);
        Button Launchpad12 = new JoystickButton(Constants.LAUNCH_PAD, 12);
        Button Launchpad13 = new JoystickButton(Constants.LAUNCH_PAD, 13);
        Button Launchpad14 = new JoystickButton(Constants.LAUNCH_PAD,14);
        Button Launchpad15 = new JoystickButton(Constants.LAUNCH_PAD,15);
        Button Launchpad16 = new JoystickButton(Constants.LAUNCH_PAD,16);


        /*GroundHatch.whileHeld(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_GROUND_HATCH, TeleOPArmAction.armStates.REV_GROUND_HATCH, Constants.ReverseButton));
        GroundCargo.whileHeld(new );
        BotHatch.whileHeld(new );
        BotCargo.whileHeld(new );
        MidHatch.whileHeld(new );
        MidCargo.whileHeld(new );
        TopHatch.whileHeld(new );
        TopCargo.whileHeld(new );
        Launchpad10.whileHeld(new );
        Launchpad11.whileHeld(new );
        Launchpad12.whileHeld(new );
        Launchpad13.whileHeld(new );*/
        Button anglePidButton = new JoystickButton(Constants.MASTER, 7);
        anglePidButton.whileHeld(Action.toCommand(new AnglePID()));
    }

}