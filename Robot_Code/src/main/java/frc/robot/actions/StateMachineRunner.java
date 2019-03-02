package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.subsystems.Drive;

public class StateMachineRunner extends Action {

    private StateMachineDescriptor state;
    private boolean running = false;

    public StateMachineRunner(StateMachineDescriptor state) {
        this.state = state;
    }

    public void onStart() {

    }

    public void onLoop() {
        running = StateMachine.runMachine(state);
    }

    public boolean isFinished() {
        return running;
    }

    public void onStop() {
        StateMachine.assertStop();
    }
}
