package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.lib.statemachine.Action;
import frc.robot.actions.*;
import frc.robot.actions.buttonactions.ModAction;
import frc.robot.actions.driveactions.AnglePID;
//import frc.robot.actions.driveactions.ForceField;
import frc.robot.autoactiongroups.StowProtocol;
import frc.robot.actions.armactions.*;
import frc.robot.actions.buttonactions.RunTestConditional;
import frc.robot.autoactiongroups.AutoTestProtocol;

import frc.robot.subsystems.Arm;

public class OI{

    public OI(){
        ////////////Scary Vision Stuff////////////
        //Button cameraSwitch = new JoystickButton(Constants.MASTER, 3);
        Button anglePidButton = new JoystickButton(Constants.MASTER, 7);
        //Button selfCheck = new JoystickButton(Constants.MASTER, 11);
        Button ForceField = new JoystickButton(Constants.MASTER, 8);

        Button ForceFieldR = new JoystickButton(Constants.MASTER, 9);

        //cameraSwitch.toggleWhenPressed(Action.toCommand(new CameraSwitchAction()));
        anglePidButton.whileHeld(Action.toCommand(new AnglePID()));
        //selfCheck.whileHeld(Action.toCommand(new RunTestConditional(new AutoTestProtocol())));
        /*
        ForceField.whileHeld(Action.toCommand(new ForceField(34)));
        ForceFieldR.whileHeld(Action.toCommand(new ForceField(15)));
        */
         ////////////End Scary Stuff////////////

        ///////////Button Declarations///////////
        //Arm Poses
        Button cargoShip = new JoystickButton(Constants.LAUNCH_PAD, 13);
        Button groundCargo = new JoystickButton(Constants.LAUNCH_PAD, 10);
        Button botCargo = new JoystickButton(Constants.LAUNCH_PAD, 12);
        Button midCargo = new JoystickButton(Constants.LAUNCH_PAD, 4);
        Button topCargo = new JoystickButton(Constants.LAUNCH_PAD, 2);
        //Climb Stuff
        Button frontClimb = new JoystickButton(Constants.LAUNCH_PAD, 16);
        Button backClimb = new JoystickButton(Constants.LAUNCH_PAD, 6);
        Button bothClimb = new JoystickButton(Constants.LAUNCH_PAD, 15);
        Button crawlerForward = new JoystickButton(Constants.MASTER, 6);
        Button crawlerBackward = new JoystickButton(Constants.MASTER, 4);
        //Cargo Manipulator
        Button cargoRollout = new JoystickButton(Constants.LAUNCH_PAD, 5);
        Button cargoShoot = new JoystickButton(Constants.LAUNCH_PAD, 11);
        Button intake = new JoystickButton(Constants.LAUNCH_PAD,14);
        //Stow/Unstow
        Button stow = new JoystickButton(Constants.LAUNCH_PAD, 8);
        Button unstow = new JoystickButton(Constants.LAUNCH_PAD, 7);
        //Safety Things
        Button autoStopButton = new JoystickButton(Constants.LAUNCH_PAD, 1);
        Button velocity = new JoystickButton(Constants.LAUNCH_PAD, 3);

        ////////////Actions Tied To Buttons////////////
        //Safety Routines
        autoStopButton.whenPressed(Action.toCommand(new AStopAction()));
        velocity.whenPressed(Action.toCommand(new VelocityAction()));
        //Climb Stuff
        /*bothClimb.whenPressed(Action.toCommand(new ModAction(new ClimbAction(DoubleSolenoid.Value.kForward, DoubleSolenoid.Value.kForward), new ClimbAction(DoubleSolenoid.Value.kReverse, DoubleSolenoid.Value.kReverse))));
        frontClimb.whenPressed(Action.toCommand(new ModAction(new ClimbAction(true, DoubleSolenoid.Value.kForward), new ClimbAction(true, DoubleSolenoid.Value.kReverse))));
        backClimb.whenPressed(Action.toCommand(new ModAction(new ClimbAction(false, DoubleSolenoid.Value.kForward), new ClimbAction(false, DoubleSolenoid.Value.kReverse))));
        */
        bothClimb.toggleWhenPressed(Action.toCommand(new climb(false)));
        bothClimb.toggleWhenPressed(Action.toCommand(new climb(true)));
        frontClimb.toggleWhenPressed(Action.toCommand(new climb(true)));
        backClimb.toggleWhenPressed(Action.toCommand(new climb(false)));
        crawlerForward.whileHeld(Action.toCommand(new Crawl(0.5)));
        crawlerBackward.whileHeld(Action.toCommand(new Crawl(-0.5)));
        //Arm Poses
        cargoShip.whenPressed(Action.toCommand(new ArmAction(Arm.ArmStates.CARGO_SHIP_CARGO)));
        groundCargo.whenPressed(Action.toCommand(new ArmAction(Arm.ArmStates.FWD_GROUND_CARGO)));
        botCargo.whenPressed(Action.toCommand(new ModAction(new ArmAction(Arm.ArmStates.FWD_LOW_CARGO), new ArmSoftCal())));
        midCargo.whenPressed(Action.toCommand(new ArmAction(Arm.ArmStates.FWD_MEDIUM_CARGO)));
        topCargo.whenPressed(Action.toCommand(new ArmAction(Arm.ArmStates.FWD_HIGH_CARGO)));
        //Cargo Manipulator
        cargoRollout.whileHeld(Action.toCommand(new ManipulatorAction(ManipulatorAction.ShotPower.SlowShoot)));
        cargoShoot.whileHeld(Action.toCommand(new ManipulatorAction(ManipulatorAction.ShotPower.Shoot)));
        intake.whileHeld(Action.toCommand(new ManipulatorAction(ManipulatorAction.ShotPower.PickUp)));

        //Stow/Unstow
        unstow.whenPressed(Action.toCommand(new UnstowArmAction()));
        stow.whenPressed(Action.toCommand(new StateMachineRunner(new StowProtocol())));

    }
}