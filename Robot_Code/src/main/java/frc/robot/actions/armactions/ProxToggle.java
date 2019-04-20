package frc.robot.actions.armactions;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class ProxToggle extends Action {
    @Override
    public void onStart() {
        System.out.println("Action Started");
        if (!Arm.getInstance().getProxExtend()) {
            System.out.println("Reverse");
            Arm.getInstance().setProxPist(DoubleSolenoid.Value.kForward);
        } else {
            System.out.println("Forward");
            Arm.getInstance().setProxPist(DoubleSolenoid.Value.kReverse);
        }
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
