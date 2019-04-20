package frc.robot.actions.buttonactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class ModAction extends Action {

    Action main, function;
    boolean sideShift;

    public ModAction(Action main, Action function) {
        this.main = main;
        this.function = function;
    }

    @Override
    public void onStart() {
        System.out.println("Activated");
        sideShift = Arm.getInstance().getSideShift();
        if (!sideShift) {
            System.out.println("Normal");
            main.onStart();
        } else {
            System.out.println("Alt");
            function.onStart();
        }
    }

    @Override
    public void onLoop() {
        if (!sideShift) {
            main.onLoop();
        } else {
            function.onLoop();
        }
    }

    @Override
    public boolean isFinished() {
        if (!sideShift) {
            return main.isFinished();
        } else {
            return function.isFinished();
        }
    }

    @Override
    public void onStop() {
        if (!sideShift) {
            main.onStop();
        } else {
            function.onStop();
        }
    }
}
