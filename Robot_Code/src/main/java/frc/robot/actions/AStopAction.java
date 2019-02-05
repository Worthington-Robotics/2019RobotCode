package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachine;

public class AStopAction extends Action {
    @Override
    public void onStart() {
        StateMachine.assertStop();
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
