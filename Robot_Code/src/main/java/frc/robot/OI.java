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
        Button CargoIn = new JoystickButton(Constants.LAUNCH_PAD, 10);
        Button CargoOut = new JoystickButton(Constants.LAUNCH_PAD, 11);
        Button HatchOut = new JoystickButton(Constants.LAUNCH_PAD, 12);
        Button Launchpad13 = new JoystickButton(Constants.LAUNCH_PAD, 13);
        Button Launchpad14 = new JoystickButton(Constants.LAUNCH_PAD,14);
        Button Launchpad15 = new JoystickButton(Constants.LAUNCH_PAD,15);
        Button Launchpad16 = new JoystickButton(Constants.LAUNCH_PAD,16);


        GroundHatch.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_GROUND_HATCH, TeleOPArmAction.armStates.REV_GROUND_HATCH, Constants.ReverseButton)));
        GroundCargo.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_GROUND_CARGO, TeleOPArmAction.armStates.REV_GROUND_CARGO, Constants.ReverseButton)));
        BotHatch.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_LOW_HATCH, TeleOPArmAction.armStates.REV_LOW_HATCH, Constants.ReverseButton)));
        BotCargo.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_LOW_CARGO, TeleOPArmAction.armStates.REV_LOW_CARGO, Constants.ReverseButton)));
        MidHatch.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_MEDIUM_HATCH, TeleOPArmAction.armStates.REV_MEDIUM_HATCH, Constants.ReverseButton)));
        MidCargo.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_MEDIUM_CARGO, TeleOPArmAction.armStates.REV_MEDIUM_CARGO, Constants.ReverseButton)));
        TopHatch.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_HIGH_HATCH, TeleOPArmAction.armStates.REV_HIGH_HATCH, Constants.ReverseButton)));
        TopCargo.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_HIGH_CARGO, TeleOPArmAction.armStates.REV_HIGH_CARGO, Constants.ReverseButton)));
        //TODO Add Cargo In/Out Functions + Hatch Out Functions
        //CargoIn.whileHeld(Action.toCommand(new ManipulatorAction(Constants.SHOOT_POWER)));
        //CargoOut.whileHeld(Action.toCommand(new ));
        //HatchOut.whileHeld(Action.toCommand(new ));
        //Launchpad13.whileHeld(Action.toCommand(new ));
        Button anglePidButton = new JoystickButton(Constants.MASTER, 7);
        anglePidButton.whileHeld(Action.toCommand(new AnglePID()));
    }

}