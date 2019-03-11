package frc.robot.actions;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Manipulator;

public class ClimbAction extends Action {
    private DoubleSolenoid.Value fPwr, bPwr;
    private boolean isOne = false;
    private boolean isFront = false;

    public ClimbAction(DoubleSolenoid.Value fpwr, DoubleSolenoid.Value bpwr) {
        fPwr = fpwr;
        bPwr = bpwr;
    }

    public ClimbAction(boolean isFront, DoubleSolenoid.Value pwr) {
        if (isFront) {
            fPwr = pwr;
            this.isFront = isFront;
        } else {
            bPwr = pwr;
        }
        isOne = true;
    }

    @Override
    public void onStart() {
        if(!isOne) {
            Manipulator.getInstance().setFrontState(fPwr);
            Manipulator.getInstance().setBackState(bPwr);
        }
        else
        {
            if(isFront)
            {Manipulator.getInstance().setFrontState(fPwr);}
            else
            {Manipulator.getInstance().setBackState(bPwr);}
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
    }
}