package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class ArmAction extends Action {
    private armStates a;
    public ArmAction(armStates armState) {
        a = armState;
    }

    @Override
    public void onStart() {
        Arm.getInstance().setArmProxPower(a.getP());
        Arm.getInstance().setArmDistPower(a.getD());
        Arm.getInstance().setArmWristPower(a.getW());


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
    public enum armStates {
        GENERAL_LOW(0,0,0),
        ROCKET_LOW_CARGO(0,0,0),
        ROCKET_MEDIUM_HATCH(0,0,0),
        ROCKET_MEDIUM_CARGO(0,0,0),
        ROCKET_HIGH_HATCH(0,0,0),
        ROCKET_HIGH_CARGO(0,0,0),
        GROUND(0,0,0);

        private double p,d,w;
        armStates(double p , double d, double w)
        {

        }
        public double getP()
        {return p;}
        public double getD()
        {return d;}
        public double getW()
        {return w;}
    }
}
