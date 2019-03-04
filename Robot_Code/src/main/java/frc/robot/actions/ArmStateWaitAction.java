package frc.robot.actions;

import frc.lib.physics.ArmModel;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;


public class ArmStateWaitAction extends Action {
    whichaction armAction;
    Arm.ArmStates armState;
    boolean end = false;
    double proxWanted, distWanted, proxCurrent, distCurrent;

    public ArmStateWaitAction(whichaction armaction, Arm.ArmStates armstate) {
        armAction = armaction;
        armState = armstate;
    }
    @Override
    public void onStart() {

    }

    @Override
    public void onLoop() {
        switch (armState) {
            case STOW_ARM: proxCurrent = Arm.ArmStates.STOW_ARM.getProx(); distCurrent = Arm.ArmStates.STOW_ARM.getDist(); break;
            case UNSTOW_ARM: proxCurrent = Arm.ArmStates.UNSTOW_ARM.getProx(); distCurrent = Arm.ArmStates.UNSTOW_ARM.getDist(); break;
            case GROUND_HATCH: proxCurrent = Arm.ArmStates.GROUND_HATCH.getProx(); distCurrent = Arm.ArmStates.GROUND_HATCH.getDist(); break;
            case FWD_LOW_CARGO: proxCurrent = Arm.ArmStates.FWD_LOW_CARGO.getProx(); distCurrent = Arm.ArmStates.FWD_LOW_CARGO.getDist(); break;
            case FWD_LOW_HATCH: proxCurrent = Arm.ArmStates.FWD_LOW_HATCH.getProx(); distCurrent = Arm.ArmStates.FWD_LOW_HATCH.getDist(); break;
            case FWD_HIGH_CARGO: proxCurrent = Arm.ArmStates.FWD_HIGH_CARGO.getProx(); distCurrent = Arm.ArmStates.FWD_HIGH_CARGO.getDist(); break;
            case FWD_HIGH_HATCH: proxCurrent = Arm.ArmStates.FWD_HIGH_HATCH.getProx(); distCurrent = Arm.ArmStates.FWD_HIGH_HATCH.getDist(); break;
            case REV_HIGH_CARGO: proxCurrent = Arm.ArmStates.REV_HIGH_CARGO.getProx(); distCurrent = Arm.ArmStates.REV_HIGH_CARGO.getDist(); break;
            case REV_HIGH_HATCH: proxCurrent = Arm.ArmStates.REV_HIGH_HATCH.getProx(); distCurrent = Arm.ArmStates.REV_HIGH_HATCH.getDist(); break;
            case FWD_GROUND_CARGO: proxCurrent =
            case FWD_MEDIUM_CARGO: proxCurrent =
            case FWD_MEDIUM_HATCH: proxCurrent =
            case REV_GROUND_CARGO: proxCurrent =
            case REV_MEDIUM_CARGO: proxCurrent =
            case REV_MEDIUM_HATCH: proxCurrent =
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {

    }
    public enum whichaction {
        ArmAction,
        TeleOPArmAction
    }
}
