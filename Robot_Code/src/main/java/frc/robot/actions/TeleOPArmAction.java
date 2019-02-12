package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class TeleOPArmAction extends Action {
    private armStates a;
    public TeleOPArmAction(armStates armState, armStates modified) {
        if (Constants.LAUNCH_PAD.getRawButton(Constants.ReverseButton)){
            a = modified;
        } else {
            a = armState;
        }

    }

    @Override
    public void onStart() {

    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {

    }
    public enum armStates {
        FWD_GROUND_HATCH(Constants.FWD_GROUND_HATCH),
        FWD_GROUND_CARGO(Constants.FWD_GROUND_CARGO),
        FWD_LOW_HATCH(Constants.FWD_LOW_HATCH),
        FWD_LOW_CARGO(Constants.FWD_LOW_CARGO),
        FWD_MEDIUM_HATCH(Constants.FWD_MEDIUM_HATCH),
        FWD_MEDIUM_CARGO(Constants.FWD_MEDUIM_CARGO),
        FWD_HIGH_HATCH(Constants.FWD_HIGH_HATCH),
        FWD_HIGH_CARGO(Constants.FWD_HIGH_CARGO),

        STRAIGHT_UP(Constants.STRAIGHT_UP),

        REV_GROUND_HATCH(Constants.REV_GROUND_HATCH),
        REV_GROUND_CARGO(Constants.REV_GROUND_CARGO),
        REV_LOW_HATCH(Constants.REV_LOW_HATCH),
        REV_LOW_CARGO(Constants.REV_LOW_CARGO),
        REV_MEDIUM_HATCH(Constants.REV_MEDIUM_HATCH),
        REV_MEDIUM_CARGO(Constants.REV_MEDUIM_CARGO),
        REV_HIGH_HATCH(Constants.REV_HIGH_HATCH),
        REV_HIGH_CARGO(Constants.REV_HIGH_CARGO);

        private Arm.ArmConfiguration config;
        armStates(Arm.ArmConfiguration conf) {
            config = conf;
        }

        public Arm.ArmConfiguration getConfig() {
            return config;
        }
    }
}
