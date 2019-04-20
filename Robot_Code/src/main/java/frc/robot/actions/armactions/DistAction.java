/*package frc.robot.actions.armactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class DistAction extends Action {
    private Arm.ArmStates a;

    public DistAction(Arm.ArmStates armState) {
        a = armState;
    }

    @Override
    public void onStart() {

        if (!Arm.getInstance().getStowed())
            Arm.getInstance().setDistPIDArmConfig(a);
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
*/