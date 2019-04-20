package frc.robot.actions.buttonactions;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMachineDescriptor;

public class RunTestConditional extends Action {

    private StateMachineDescriptor state;
    private boolean running = false;

    public RunTestConditional(StateMachineDescriptor state) {
        this.state = state;
    }

    @Override
    public void onStart() {
        if (DriverStation.getInstance().isTest()) {
            StateMachine.runMachine(state);
        }
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
        StateMachine.assertStop();
    }
}
