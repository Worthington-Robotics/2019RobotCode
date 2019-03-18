package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.lib.util.DriveSignal;
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
        if(dis < mDis && dis > 0)
        {
            Drive.getInstance().setOpenLoop(new DriveSignal(0,0));
        }
    }

    public boolean isFinished() {
        return false;
    }

    public void onStop() {

    }
}
