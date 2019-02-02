package frc.robot.actions;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Alien;

public class AlienAction extends Action {
       public AlienAction()
    {
    }
    @Override
    public void onStart() { Alien.getInstance().setAlienState(DoubleSolenoid.Value.kReverse);


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
        Alien.getInstance().setAlienState(DoubleSolenoid.Value.kForward);
        //Alien.getInstance().setAlienState(DoubleSolenoid.Value.kOff);
    }
}
