/*package frc.robot.actions.waitactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;


public class ArmStateWaitAction extends Action {
    Arm.ArmStates armState;
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

    }
    @Override
    public boolean isFinished() {
        if ((armState.getProx() >= Arm.getInstance().getProxPoint() - kEpsilon &&
                armState.getProx() <= Arm.getInstance().getProxPoint() + kEpsilon) &&
                (armState.getDist() >= Arm.getInstance().getDistPoint() - kEpsilon &&
                armState.getDist() <= Arm.getInstance().getDistPoint() + kEpsilon)) return true;
        return false;
    }

    @Override
    public void onStop() {
    }
}
*/