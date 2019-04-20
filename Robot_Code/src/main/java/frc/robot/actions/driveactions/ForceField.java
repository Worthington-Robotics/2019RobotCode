/*package frc.robot.actions.driveactions;

import frc.lib.statemachine.Action;
import frc.lib.util.DriveSignal;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.vision.Vision;

public class ForceField extends Action  {
    private double mDis;
    public ForceField(double dis)
    {
        mDis = dis;
    }

    public void onStart() {

    }

    public void onLoop() {
        double dis = Vision.getInstance().getDis();
        if(Arm.getInstance().getArmState().equals(Arm.ArmStates.CARGO_SHIP_CARGO))
            mDis = 17;
        else if(Arm.getInstance().getArmState().equals(Arm.ArmStates.FWD_HIGH_CARGO) || Arm.getInstance().getArmState().equals(Arm.ArmStates.FWD_MEDIUM_CARGO) || Arm.getInstance().getArmState().equals(Arm.ArmStates.FWD_LOW_CARGO))
            mDis = 37;
        else
            mDis = -2;

        if(dis < mDis && dis > 0)
        {
            Drive.getInstance().setAnglePidLoop(new DriveSignal(0,0));
        }
    }

    public boolean isFinished() {
        return false;
    }

    public void onStop() {
        Drive.getInstance().setOpenLoop(new DriveSignal(0,0));

    }
}
*/