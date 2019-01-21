package frc.lib.statemachine;

import java.util.concurrent.ConcurrentLinkedQueue;

public class StateMachineDescriptor {
    private ConcurrentLinkedQueue<ActionGroup> Queuedstates;

    public StateMachineDescriptor() {
        Queuedstates = new ConcurrentLinkedQueue<>();
    }

    public void addSequntal(Action action, long timeout_ms) {
        Queuedstates.add(new ActionGroup(action, timeout_ms));
    }

    public void addParallel(Action[] action, long timeout_ms) {
        Queuedstates.add(new ActionGroup(action, timeout_ms));
    }

    public ConcurrentLinkedQueue getStates() {
        return Queuedstates;
    }
}
