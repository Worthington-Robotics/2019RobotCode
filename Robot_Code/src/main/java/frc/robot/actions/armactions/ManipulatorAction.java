package frc.robot.actions.armactions;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class ManipulatorAction extends Action {
    private ShotPower speed;

    public ManipulatorAction(ShotPower shotPower) {
        speed = shotPower;
    }

    public void onStart() {
        Manipulator.getInstance().setShotPower(speed.shotpower);
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
        Manipulator.getInstance().setShotPower(ShotPower.Stop.shotpower);
    }

    public enum ShotPower {
        Shoot(Constants.SHOOT_POWER),
        SlowShoot(Constants.SLOW_SHOOT_POWER),
        PickUp(Constants.PICKUP_POWER),
        Stop(Constants.STOP_POWER);
        private double shotpower;

        ShotPower(double power) {
            shotpower = power;
        }
    }
}

