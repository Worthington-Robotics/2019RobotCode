package frc.robot.actions.armactions;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmSoftCal extends Action {

    @Override
    public void onStart() {
        double absolutes = Arm.getInstance().getAbsolute();
        Constants.DIST_ABSOLUTE_ZERO = absolutes - Arm.PistonArmStates.STOW_ARM.getDist();
        Arm.getInstance().resetArmMod();
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


