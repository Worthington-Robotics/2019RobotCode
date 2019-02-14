package frc.robot.actions;

import edu.wpi.first.wpilibj.Spark;
import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class ClimbAction extends Action {
    private boolean reversed;
    private double motorpower;
    public ClimbAction(boolean reversed, double motorpower) {
        this.reversed = reversed;
        this.motorpower = motorpower;
    }
    @Override
    public void onStart() {
        /*Drive.getInstance().setReversed(reversed);
        Drive.getInstance().setMotorPower(motorpower);*/
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
    /*Drive.getInstance().setMotorPower(0);*/
    }
}
