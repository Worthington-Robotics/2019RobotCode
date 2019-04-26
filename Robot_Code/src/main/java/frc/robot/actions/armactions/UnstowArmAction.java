package frc.robot.actions.armactions;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;


public class UnstowArmAction extends Action {
    private Arm.PistonArmStates a;

    public UnstowArmAction() {
        a = Arm.PistonArmStates.UNSTOW_ARM;
    }

    @Override
    public void onStart() {
        Manipulator.getInstance().setFrontState(DoubleSolenoid.Value.kForward);
        Arm.getInstance().setPistPIDArmConfig(a);
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
