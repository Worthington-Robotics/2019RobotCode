package frc.robot.actions.armactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class VelocityAction extends Action {

    @Override
    public void onStart() {
        Arm.getInstance().setVelocitymConfig();
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
