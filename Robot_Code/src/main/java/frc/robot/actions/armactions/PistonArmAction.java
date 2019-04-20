package frc.robot.actions.armactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class PistonArmAction extends Action {
    private Arm.PistonArmStates a;

    public PistonArmAction(Arm.PistonArmStates armState) {
        a = armState;
    }

    @Override
    public void onStart() {

        if (!Arm.getInstance().getStowed())
            Arm.getInstance().setPistPIDArmConfig(a);
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
