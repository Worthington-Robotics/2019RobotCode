package frc.robot.actions;

import edu.wpi.first.wpilibj.Spark;
import frc.lib.statemachine.Action;
import frc.robot.Constants;

public class ClimbAction extends Action {
    boolean reversed;
    double motorpower;
    Spark climbLeft, climbRight;
    private ClimbAction() {
        climbLeft = new Spark(Constants.LEFT_CLIMB_ID);
        climbRight = new Spark(Constants.RIGHT_CLIMB_ID);
    }

    public void setClimber(boolean reversed, double motorpower) {
        climbLeft.set(motorpower);
        climbRight.set(motorpower);
    }
    @Override
    public void onStart() {

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
