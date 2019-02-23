package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Drive;

public class ClimbAction extends Action {
    private double motorpower;
    public ClimbAction(double motorpower) {
        this.motorpower = motorpower;
    }
    @Override
    public void onStart() {
        Drive.getInstance().setMotorPower(motorpower);
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
        Drive.getInstance().setMotorPower(0);
    }
}
