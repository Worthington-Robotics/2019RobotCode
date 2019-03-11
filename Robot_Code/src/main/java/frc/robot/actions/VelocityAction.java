package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class VelocityAction extends Action {

    @Override
    public void onStart() {
        Arm.getInstance().setVelocitymConfig(0,0);
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
