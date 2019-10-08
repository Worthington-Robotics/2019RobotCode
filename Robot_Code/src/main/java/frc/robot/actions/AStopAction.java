package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachine;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;

public class AStopAction extends Action {
    boolean finished = false;

    @Override
    public void onStart() {
        StateMachine.assertStop();
        Drive.getInstance().overrideTrajectory(true);
        Arm.getInstance().setVelocitymConfig();
//        Manipulator.getInstance().setElevatorPower(0);
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

    public static class Lock {
    }
}
