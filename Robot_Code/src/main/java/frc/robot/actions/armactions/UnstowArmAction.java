package frc.robot.actions.armactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;


public class UnstowArmAction extends Action {
    private Arm.ArmStates a;
    public UnstowArmAction() {
        a = Arm.ArmStates.UNSTOW_ARM;
    }

    @Override
    public void onStart() {
        Arm.getInstance().setPIDArmConfig(a);
        Arm.getInstance().setStowed(false);
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {

    }

}
