package frc.robot.actions;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Manipulator;

public class climb extends Action {
    private boolean IsFront;
    public climb(boolean isFront){ IsFront = isFront;
    }
    @Override
    public void onStart() {
            Manipulator.getInstance().setFrontState(DoubleSolenoid.Value.kForward);


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
        Manipulator.getInstance().setFrontState(DoubleSolenoid.Value.kReverse);
    }
}
