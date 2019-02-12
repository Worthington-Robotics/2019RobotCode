package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.lib.statemachine.Action;
import frc.robot.actions.AnglePID;
import frc.robot.actions.ClimbAction;
import frc.robot.actions.TeleOPArmAction;

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
        Button ClimbFullUp = new JoystickButton(Constants.LAUNCH_PAD, 13);
        Button ClimbFullDown = new JoystickButton(Constants.LAUNCH_PAD,14);
        Button Launchpad15 = new JoystickButton(Constants.LAUNCH_PAD,15);
        Button Launchpad16 = new JoystickButton(Constants.LAUNCH_PAD,16);


        GroundHatch.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_GROUND_HATCH, TeleOPArmAction.armStates.REV_GROUND_HATCH)));
        GroundCargo.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_GROUND_CARGO, TeleOPArmAction.armStates.REV_GROUND_CARGO)));
        BotHatch.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_LOW_HATCH, TeleOPArmAction.armStates.REV_LOW_HATCH)));
        BotCargo.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_LOW_CARGO, TeleOPArmAction.armStates.REV_LOW_CARGO)));
        MidHatch.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_MEDIUM_HATCH, TeleOPArmAction.armStates.REV_MEDIUM_HATCH)));
        MidCargo.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_MEDIUM_CARGO, TeleOPArmAction.armStates.REV_MEDIUM_CARGO)));  
        TopHatch.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_HIGH_HATCH, TeleOPArmAction.armStates.REV_HIGH_HATCH)));
        TopCargo.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_HIGH_CARGO, TeleOPArmAction.armStates.REV_HIGH_CARGO)));
        //TODO Add Cargo In/Out Functions + Hatch Out Functions
        //CargoIn.whileHeld(Action.toCommand(new ManipulatorAction(Constants.SHOOT_POWER)));
        //CargoOut.whileHeld(Action.toCommand(new ));
        //HatchOut.whileHeld(Action.toCommand(new ));
        //TODO Add Climbing
        ClimbFullUp.whileHeld(Action.toCommand(new ClimbAction(false,Constants.CLIMB_POWER)));
        ClimbFullDown.whileHeld(Action.toCommand(new ClimbAction(true,Constants.CLIMB_POWER)));
        Button anglePidButton = new JoystickButton(Constants.MASTER, 7);
        anglePidButton.whileHeld(Action.toCommand(new AnglePID()));
    }

}