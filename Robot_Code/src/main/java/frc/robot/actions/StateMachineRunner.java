package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMachineDescriptor;

public class StateMachineRunner extends Action {

    private StateMachineDescriptor state;
    private boolean running = false;

    public StateMachineRunner(StateMachineDescriptor state) {
        this.state = state;
    }

    public void onStart() {
        StateMachine.runMachine(state);
    }

    public void onLoop() {

    }

    public boolean isFinished() {
        return !StateMachine.isRunning();//running;
    }

    public void onStop() {
        StateMachine.assertStop();
    }
}
