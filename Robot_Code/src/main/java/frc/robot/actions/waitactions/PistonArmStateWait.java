package frc.robot.actions.waitactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class PistonArmStateWait extends Action {
    Arm.PistonArmStates armState;
    boolean end;
    double kEpsilon;

    public PistonArmStateWait(Arm.PistonArmStates armState, double kEpsilon) {
        this.end = false;
        this.armState = armState;
        this.kEpsilon = kEpsilon;
    }

    @Override
    public void onStart() {

    }

    @Override
    public void onLoop() {
        if ((Math.abs(armState.getDist() - Arm.getInstance().getDistPoint()) <= kEpsilon)) {
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
