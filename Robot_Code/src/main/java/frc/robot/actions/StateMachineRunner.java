package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMachineDescriptor;

public class StateMachineRunner extends Action {
    private StateMachineDescriptor state;
    private boolean running = false;
    public StateMachineRunner(StateMachineDescriptor state)
    {
        this.state = state;
    }

    @Override
    public void onStart() {

    }

    @Override
    public void onLoop() {
        running = StateMachine.runMachine(state);
    }

    @Override
    public boolean isFinished() {
        return running;
    }

    @Override
    public void onStop() {

    }
}
