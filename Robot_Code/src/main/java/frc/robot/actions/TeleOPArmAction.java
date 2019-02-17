package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class TeleOPArmAction extends Action {
    private Arm.ArmStates a;
    public TeleOPArmAction(Arm.ArmStates armState, Arm.ArmStates modified) {
        if (Constants.LAUNCH_PAD.getRawButton(9)){
            a = modified;
        } else {
            a = armState;
        }

    }

    @Override
    public void onStart() {
        Arm.getInstance().setPIDArmConfig(a);
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {

    }
}
