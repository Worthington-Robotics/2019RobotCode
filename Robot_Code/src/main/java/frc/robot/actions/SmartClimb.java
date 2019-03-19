package frc.robot.actions;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Manipulator;

public class SmartClimb extends Action {
    /**
     * code to run on action start
     */
    private boolean isFront;
    public SmartClimb(boolean isFront)
    {
        this.isFront = isFront;
    }
    public void onStart() {
        if(isFront)
        {
            if(Manipulator.getInstance().GetFrontState().equals(DoubleSolenoid.Value.kForward))
                Manipulator.getInstance().setFrontState(DoubleSolenoid.Value.kReverse);
            else
                Manipulator.getInstance().setFrontState(DoubleSolenoid.Value.kForward);
        }
        else
        {

            if(Manipulator.getInstance().GetBackState().equals(DoubleSolenoid.Value.kForward))
                Manipulator.getInstance().setBackState(DoubleSolenoid.Value.kReverse);
            else
                Manipulator.getInstance().setBackState(DoubleSolenoid.Value.kForward);
        }
    }

    /**
     * code to run while action loops
     * <p>approx every 20 miliseconds
     */
    public void onLoop() {

    }

    /**
     * method that tells the state machine the action is finished earlier than the scheduler
     *
     * @return true when action is ready to self terminate
     */
    public boolean isFinished() {
        return false;
    }

    /**
     * code to run when the action has ben called by the state machine to stop
     */
    public void onStop() {

    }
}
