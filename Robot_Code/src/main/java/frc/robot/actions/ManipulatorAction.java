package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.Constants;

public class ManipulatorAction extends Action {
    private ShotPower speed;

    private static final ManipulatorAction m_instance = new ManipulatorAction(ShotPower.Stop);

    public static ManipulatorAction get_instance() {
        return m_instance;
    }

    public void setShotPower(ShotPower speed) {
        this.speed = speed;
    }

    public ManipulatorAction(ShotPower Speed) {
        speed = Speed;
    }

    public void onStart() {
        ManipulatorAction.get_instance().setShotPower(ShotPower.Stop);

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
        ManipulatorAction.get_instance().setShotPower(ShotPower.Stop);

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

