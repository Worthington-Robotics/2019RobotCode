package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;


public class ArmStateWaitAction extends Action {
    Arm.ArmStates armState;
    boolean end = false;
    double kEpsilon;

    public ArmStateWaitAction(Arm.ArmStates armState, double kEpsilon) {
        this.armState = armState;
        this.kEpsilon = kEpsilon;
    }

    @Override
    public void onStart() {

    }

    @Override
    public void onLoop() {
        if ((Math.abs(armState.getProx() - Arm.getInstance().getProxPoint()) <= kEpsilon) &&
                (Math.abs(armState.getDist() - Arm.getInstance().getDistPoint()) <= kEpsilon)) {
            end = true;
        }
    }

    @Override
    public boolean isFinished() {
        return end;
    }

    @Override
    public void onStop() {
    }
}
