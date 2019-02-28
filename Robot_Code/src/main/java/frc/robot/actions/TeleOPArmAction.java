package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class TeleOPArmAction extends Action {
    private Arm.ArmStates armState, modified;
    public TeleOPArmAction(Arm.ArmStates armState, Arm.ArmStates modified) {
        this.armState = armState;
        this.modified = modified;
    }

    @Override
    public void onStart() {
        if (!Arm.getInstance().getSideShift()){
            Arm.getInstance().setPIDArmConfig(armState);
        } else {
            Arm.getInstance().setPIDArmConfig(modified);
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
