package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmSoftCal extends Action {

    @Override
    public void onStart() {
        double[] absolutes  =  Arm.getInstance().getAbsolute();
        Constants.PROX_ABSOLUTE_ZERO = absolutes[0] - Arm.ArmStates.FWD_LOW_CARGO.getProx();
        Constants.DIST_ABSOLUTE_ZERO = absolutes[1] - Arm.ArmStates.FWD_LOW_CARGO.getDist();
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
