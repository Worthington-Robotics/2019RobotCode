package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class ModAction extends Action {

    Action main, function;

    public ModAction(Action main, Action function){
        this.main = main;
        this.function = function;
    }

    @Override
    public void onStart() {
        if(!Arm.getInstance().getSideShift()){
            main.onStart();
        }
        else{
            function.onStart();
        }
    }

    @Override
    public void onLoop() {
        if(!Arm.getInstance().getSideShift()){
            main.onLoop();
        }
        else{
            function.onLoop();
        }
    }

    @Override
    public boolean isFinished() {
        if(!Arm.getInstance().getSideShift()){
            return main.isFinished();
        }
        else{
            return function.isFinished();
        }
    }

    @Override
    public void onStop() {
        if(!Arm.getInstance().getSideShift()){
            main.onStop();
        }
        else{
            function.onStop();
        }
    }
}
