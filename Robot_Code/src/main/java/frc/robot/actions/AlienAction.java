package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Alien;

public class AlienAction extends Action {
    private double AlienState;
    public AlienAction(double state)
    {
        AlienState = state;
    }
    @Override
    public void onStart() { Alien.getInstance().setAlienState(AlienState); }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
    Alien.getInstance().setAlienState(0);
    }
}
