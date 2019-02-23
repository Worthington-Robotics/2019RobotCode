package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;


public class ArmAction extends Action {
    private Arm.ArmStates a;
    public ArmAction(Arm.ArmStates armState) {
        a = armState;
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
