package frc.robot.actions.armactions;


import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachine;
import frc.robot.subsystems.Arm;

public class StowArmAction extends Action {
    private Arm.PistonArmStates a;

    public StowArmAction() {
        a = Arm.PistonArmStates.STOW_ARM;
    }

    @Override
    public void onStart() {
        Arm.getInstance().setPistPIDArmConfig(a);
        Arm.getInstance().setStowed(true);
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
        StateMachine.assertStop();
    }

}
