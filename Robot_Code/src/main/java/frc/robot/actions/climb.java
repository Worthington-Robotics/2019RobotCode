package frc.robot.actions;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Manipulator;

public class climb extends Action {
    private boolean IsFront;
    public climb(boolean isFront){
        IsFront = isFront;
    }
    @Override
    public void onStart() {
        if(IsFront)
        {
            Manipulator.getInstance().setFrontState(DoubleSolenoid.Value.kForward);
        }
        else
        {
            Manipulator.getInstance().setBackState(DoubleSolenoid.Value.kForward);
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
        if(IsFront)
        {
            Manipulator.getInstance().setFrontState(DoubleSolenoid.Value.kReverse);
        }
        else
        {
            Manipulator.getInstance().setBackState(DoubleSolenoid.Value.kReverse);
        }

    }
}
