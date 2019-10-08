package frc.robot.actions.armactions;


import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;

public class StowArmAction extends Action {
    private Arm.PistonArmStates a;

    public StowArmAction() {
        a = Arm.PistonArmStates.STOW_ARM;
    }

    @Override
    public void onStart() {
        Arm.getInstance().eh();
        Arm.getInstance().armDist.set(ControlMode.PercentOutput, -.75);
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        if (Arm.getInstance().armDist.getSensorCollection().isRevLimitSwitchClosed()) {
            Arm.getInstance().setStowed(true);
            Manipulator.getInstance().setLockState(DoubleSolenoid.Value.kReverse);
            return true;
        }
        return false;
    }

    @Override
    public void onStop() {
        Arm.getInstance().safeMode();
    }

}
