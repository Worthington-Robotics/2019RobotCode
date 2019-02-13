package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.CargoMani;

public class ManipulatorAction extends Action {
    private ShotPower speed;

    public Action setShotPower(ShotPower shotPower) {
        speed = shotPower;
        return null;
    }

    public void onStart() {
        CargoMani.getInstance().setShotPower(speed.shotpower);
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
        CargoMani.getInstance().setShotPower(ShotPower.Stop.shotpower);
    }

    public enum ShotPower {
        Shoot(Constants.SHOOT_POWER),
        PickUp(Constants.PICKUP_POWER),
        Stop(Constants.STOP_POWER);
        private double shotpower;

        ShotPower(double power) {
            shotpower = power;
        }
    }
}

