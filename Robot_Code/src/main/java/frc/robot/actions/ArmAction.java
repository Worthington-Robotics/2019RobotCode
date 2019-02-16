package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;


public class ArmAction extends Action {
    private Arm.armStates a;
    public ArmAction(Arm.armStates armState) {
        a = armState;
    }

    @Override
    public void onStart() {
        Arm.getInstance().setPIDArmConfig(a.getConfig());
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
