package frc.robot.actions;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Manipulator;

public class Lock extends Action {
    @Override
    public void onStart() {
        Manipulator.getInstance().setLockState(DoubleSolenoid.Value.kReverse);


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
        Manipulator.getInstance().setLockState(DoubleSolenoid.Value.kForward);
    }
}

