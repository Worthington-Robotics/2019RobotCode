package frc.robot.actions.armactions;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class TeleOPArmAction extends Action {
    private Arm.ArmStates armState, modified;
    public TeleOPArmAction(Arm.ArmStates armState, Arm.ArmStates modified) {
        this.armState = armState;
        this.modified = modified;
    }
    public Arm.ArmStates getArmState() {
        if (!Arm.getInstance().getSideShift()) {
            return armState;
        } else {
            return modified;
        }
    }

    @Override
    public void onStart() {
        if (!Arm.getInstance().getStowed()) {
            if (!Arm.getInstance().getSideShift()) {
                Arm.getInstance().setPIDArmConfig(armState);
            } else {
                Arm.getInstance().setPIDArmConfig(modified);
            }
        }
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
