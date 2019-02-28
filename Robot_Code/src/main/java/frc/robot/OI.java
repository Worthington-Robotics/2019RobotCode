package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.lib.statemachine.Action;
import frc.robot.actions.*;
import frc.robot.autoactiongroups.AutoDockStateMachine;
import frc.robot.subsystems.Arm;

public class OI{

    public OI(){
        Button AllAction = new JoystickButton(Constants.MASTER, 1);
        Button Vision = new JoystickButton(Constants.MASTER, 8);
        Button anglePidButton = new JoystickButton(Constants.MASTER, 7);
        Button cameraSwitch = new JoystickButton(Constants.MASTER, 3);

        anglePidButton.whileHeld(Action.toCommand(new AnglePID()));
        Vision.whileHeld(Action.toCommand(new VisionTra()));
        AllAction.whileHeld(Action.toCommand(new StateMachineRunner(new AutoDockStateMachine())));
        cameraSwitch.whileHeld(Action.toCommand(new CameraSwitchAction()));

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

        GroundHatch.whenPressed(Action.toCommand(new ArmAction(Arm.ArmStates.GROUND_HATCH)));
        GroundCargo.whenPressed(Action.toCommand(new TeleOPArmAction(Arm.ArmStates.FWD_GROUND_CARGO, Arm.ArmStates.REV_GROUND_CARGO)));
        BotCargo.whenPressed(Action.toCommand(new ArmAction(Arm.ArmStates.FWD_LOW_CARGO)));
        BotHatch.whenPressed(Action.toCommand(new ArmAction(Arm.ArmStates.FWD_LOW_HATCH)));
        MidHatch.whenPressed(Action.toCommand(new TeleOPArmAction(Arm.ArmStates.FWD_MEDIUM_HATCH, Arm.ArmStates.REV_MEDIUM_HATCH)));
        MidCargo.whenPressed(Action.toCommand(new TeleOPArmAction(Arm.ArmStates.FWD_MEDIUM_CARGO, Arm.ArmStates.REV_MEDIUM_CARGO)));
        TopHatch.whenPressed(Action.toCommand(new TeleOPArmAction(Arm.ArmStates.FWD_HIGH_HATCH, Arm.ArmStates.REV_HIGH_HATCH)));
        TopCargo.whenPressed(Action.toCommand(new TeleOPArmAction(Arm.ArmStates.FWD_HIGH_CARGO, Arm.ArmStates.REV_HIGH_CARGO)));
        CargoIn.whileHeld(Action.toCommand(new ManipulatorAction(ManipulatorAction.ShotPower.PickUp)));
        CargoOut.whileHeld(Action.toCommand(new ManipulatorAction(ManipulatorAction.ShotPower.Shoot)));
        HatchOut.whileHeld(Action.toCommand(new AlienAction()));
        ClimbFullUp.whileHeld(Action.toCommand(new ClimbAction(-Constants.CLIMB_POWER)));
        ClimbFullDown.whileHeld(Action.toCommand(new ClimbAction(Constants.CLIMB_POWER)));
        AutoStopButton.whenPressed(Action.toCommand(new AStopAction()));
        Stow.whenPressed(Action.toCommand(new ArmAction(Arm.ArmStates.STOW_ARM)));

    }

}