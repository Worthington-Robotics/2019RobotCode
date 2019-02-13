package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachine;

public class AStopAction extends Action {
    boolean finished = false;
    @Override
    public void onStart() {
        StateMachine.assertStop();
        finished = true;
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void onStop() {

    }
}
