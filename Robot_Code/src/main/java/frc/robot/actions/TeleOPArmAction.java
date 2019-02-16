package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class TeleOPArmAction extends Action {
    private Arm.armStates a;
    public TeleOPArmAction(Arm.armStates armState, Arm.armStates modified) {
        if (Constants.LAUNCH_PAD.getRawButton(9)){
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
}
