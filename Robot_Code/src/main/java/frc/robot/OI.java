package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.lib.statemachine.Action;
import frc.robot.actions.*;
import frc.robot.actions.buttonactions.ModAction;
import frc.robot.autoactiongroups.StowProtocol;
import frc.robot.actions.armactions.*;
import frc.robot.actions.buttonactions.RunTestConditional;
import frc.robot.autoactiongroups.AutoTestProtocol;

import frc.robot.subsystems.Arm;

public class OI{

    public OI(){
        //Button allAction = new JoystickButton(Constants.MASTER, 1);
        Button cameraSwitch = new JoystickButton(Constants.MASTER, 3);
        Button anglePidButton = new JoystickButton(Constants.MASTER, 7);
        //Button vision = new JoystickButton(Constants.MASTER, 8);
        Button selfCheck = new JoystickButton(Constants.MASTER, 12);

        cameraSwitch.toggleWhenPressed(Action.toCommand(new CameraSwitchAction()));
        anglePidButton.whileHeld(Action.toCommand(new AnglePID()));
        selfCheck.whileHeld(Action.toCommand(new RunTestConditional(new AutoTestProtocol())));

        Button groundHatch = new JoystickButton(Constants.LAUNCH_PAD, 13);
        Button groundCargo = new JoystickButton(Constants.LAUNCH_PAD, 10);
        Button botHatch = new JoystickButton(Constants.LAUNCH_PAD, 15);
        Button botCargo = new JoystickButton(Constants.LAUNCH_PAD, 12);
        Button midHatch = new JoystickButton(Constants.LAUNCH_PAD, 16);
        Button midCargo = new JoystickButton(Constants.LAUNCH_PAD, 4);
        Button topHatch = new JoystickButton(Constants.LAUNCH_PAD, 6);
        Button topCargo = new JoystickButton(Constants.LAUNCH_PAD, 2);
        Button cargoIn = new JoystickButton(Constants.LAUNCH_PAD, 5);
        Button cargoOut = new JoystickButton(Constants.LAUNCH_PAD, 11);
        Button hatchOut = new JoystickButton(Constants.LAUNCH_PAD,14);
        Button climbFullUp = new JoystickButton(Constants.LAUNCH_PAD, 7);
        Button climbFullDown = new JoystickButton(Constants.LAUNCH_PAD, 8);
        Button autoStopButton = new JoystickButton(Constants.LAUNCH_PAD, 1);
        Button stow = new JoystickButton(Constants.LAUNCH_PAD, 3);


        groundHatch.whenPressed(Action.toCommand(new TeleOPArmAction(Arm.ArmStates.FWD_GROUND_CARGO, Arm.ArmStates.GROUND_HATCH)));
        groundCargo.whenPressed(Action.toCommand(new TeleOPArmAction(Arm.ArmStates.FWD_GROUND_CARGO, Arm.ArmStates.GROUND_HATCH)));
        botCargo.whenPressed(Action.toCommand(new ModAction(new ArmAction(Arm.ArmStates.FWD_LOW_CARGO), new ArmSoftCal())));
        botHatch.whenPressed(Action.toCommand(new ArmAction(Arm.ArmStates.FWD_LOW_HATCH)));
        midHatch.whenPressed(Action.toCommand(new TeleOPArmAction(Arm.ArmStates.FWD_MEDIUM_HATCH, Arm.ArmStates.REV_MEDIUM_HATCH)));
        midCargo.whenPressed(Action.toCommand(new TeleOPArmAction(Arm.ArmStates.FWD_MEDIUM_CARGO, Arm.ArmStates.REV_MEDIUM_CARGO)));
        topHatch.whenPressed(Action.toCommand(new TeleOPArmAction(Arm.ArmStates.FWD_HIGH_HATCH, Arm.ArmStates.REV_HIGH_HATCH)));
        topCargo.whenPressed(Action.toCommand(new TeleOPArmAction(Arm.ArmStates.FWD_HIGH_CARGO, Arm.ArmStates.REV_HIGH_CARGO)));
        cargoIn.whileHeld(Action.toCommand(new ManipulatorAction(ManipulatorAction.ShotPower.PickUp)));
        cargoOut.whileHeld(Action.toCommand(new ManipulatorAction(ManipulatorAction.ShotPower.Shoot)));
        hatchOut.whileHeld(Action.toCommand(new AlienAction()));
        climbFullUp.whileHeld(Action.toCommand(new ArmAction(Arm.ArmStates.STOW_ARM)));
        climbFullDown.whileHeld(Action.toCommand(new ClimbAction(Constants.CLIMB_POWER)));
        autoStopButton.whenPressed(Action.toCommand(new AStopAction()));
        stow.whileHeld(Action.toCommand(new ModAction(new UnstowArmAction(), new StateMachineRunner(new StowProtocol()))));


    }
}