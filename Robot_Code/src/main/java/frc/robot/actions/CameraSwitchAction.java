package frc.robot.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.statemachine.Action;

public class CameraSwitchAction extends Action {

    @Override
    public void onStart() {
        SmartDashboard.putString("CameraSelection", "Back");
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
        SmartDashboard.putString("CameraSelection", "Front");
    }

}
