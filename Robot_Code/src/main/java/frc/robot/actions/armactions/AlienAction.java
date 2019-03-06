package frc.robot.actions.armactions;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Manipulator;

public class AlienAction extends Action {
    public AlienAction() {
    }

    @Override
    public void onStart() {
        Manipulator.getInstance().setAlienState(DoubleSolenoid.Value.kReverse);
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
        Manipulator.getInstance().setAlienState(DoubleSolenoid.Value.kForward);
    }
}
