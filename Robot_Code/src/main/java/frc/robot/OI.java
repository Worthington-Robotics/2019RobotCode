package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.*;
import frc.robot.autoactiongroups.AutoDockStateMachine;

public class OI{

    public OI(){
        Button AllAction = new JoystickButton(Constants.MASTER, 1);
        Button Vision = new JoystickButton(Constants.MASTER, 8);
        Button anglePidButton = new JoystickButton(Constants.MASTER, 7);
        Button GroundHatch = new JoystickButton(Constants.LAUNCH_PAD, 13);
        Button GroundCargo = new JoystickButton(Constants.LAUNCH_PAD, 10);
        Button BotHatch = new JoystickButton(Constants.LAUNCH_PAD, 15);
        Button BotCargo = new JoystickButton(Constants.LAUNCH_PAD, 12);
        Button MidHatch = new JoystickButton(Constants.LAUNCH_PAD, 16);
        Button MidCargo = new JoystickButton(Constants.LAUNCH_PAD, 4);
        Button TopHatch = new JoystickButton(Constants.LAUNCH_PAD, 6);
        Button TopCargo = new JoystickButton(Constants.LAUNCH_PAD, 2);
        Button CargoIn = new JoystickButton(Constants.LAUNCH_PAD, 5);
        Button CargoOut = new JoystickButton(Constants.LAUNCH_PAD, 11);
        Button HatchOut = new JoystickButton(Constants.LAUNCH_PAD,14);
        Button ClimbFullUp = new JoystickButton(Constants.LAUNCH_PAD, 7);
        Button ClimbFullDown = new JoystickButton(Constants.LAUNCH_PAD, 8);
        Button AutoStopButton = new JoystickButton(Constants.LAUNCH_PAD, 1);
        Button Stow = new JoystickButton(Constants.LAUNCH_PAD, 3);

        GroundHatch.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.REV_GROUND_CARGO, TeleOPArmAction.armStates.REV_GROUND_CARGO)));
        GroundCargo.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.REV_GROUND_HATCH, TeleOPArmAction.armStates.REV_GROUND_HATCH)));
        BotCargo.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_LOW_CARGO, TeleOPArmAction.armStates.FWD_LOW_CARGO)));
        BotHatch.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_LOW_HATCH, TeleOPArmAction.armStates.FWD_LOW_HATCH)));
        MidHatch.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_MEDIUM_HATCH, TeleOPArmAction.armStates.REV_MEDIUM_HATCH)));
        MidCargo.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_MEDIUM_CARGO, TeleOPArmAction.armStates.REV_MEDIUM_CARGO)));  
        TopHatch.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_HIGH_HATCH, TeleOPArmAction.armStates.REV_HIGH_HATCH)));
        TopCargo.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_HIGH_CARGO, TeleOPArmAction.armStates.REV_HIGH_CARGO)));
        CargoIn.whileHeld(Action.toCommand(new ManipulatorAction(ManipulatorAction.ShotPower.PickUp)));
        CargoOut.whileHeld(Action.toCommand(new ManipulatorAction(ManipulatorAction.ShotPower.Shoot)));
        HatchOut.whileHeld(Action.toCommand(new AlienAction()));
        ClimbFullUp.whileHeld(Action.toCommand(new ClimbAction(false,Constants.CLIMB_POWER)));
        ClimbFullDown.whileHeld(Action.toCommand(new ClimbAction(true,Constants.CLIMB_POWER)));
        AutoStopButton.whileHeld(Action.toCommand(new AStopAction()));
        Stow.whileHeld(Action.toCommand(new TeleOPArmAction(TeleOPArmAction.armStates.FWD_STOW_ARM, TeleOPArmAction.armStates.FWD_STOW_ARM) ));
        anglePidButton.whileHeld(Action.toCommand(new AnglePID()));
        Vision.whileHeld(Action.toCommand(new VisionTra()));
        AllAction.whileHeld(Action.toCommand(new StateMachineRunner(new AutoDockStateMachine())));
    }

}