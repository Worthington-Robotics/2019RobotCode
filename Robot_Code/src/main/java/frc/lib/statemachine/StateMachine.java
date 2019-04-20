package frc.lib.statemachine;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class StateMachine {

    private static final AtomicInteger state = new AtomicInteger(-1);
    private static final AtomicBoolean wantStop = new AtomicBoolean(true);
    private static final AtomicBoolean stateLock = new AtomicBoolean(false);
    private volatile static ConcurrentLinkedQueue<ActionGroup> queuedStates;
    private volatile static ActionGroup currentState;
    private volatile static double t_start;
    private static final double delay = 0.020;

    private static final Runnable Man = () -> {
        try {
            state.set(0);
            SmartDashboard.putNumber("StateMachine/state", state.get());
            if (queuedStates == null) {
                state.set(-2);
                SmartDashboard.putNumber("StateMachine/state", state.get());
            } else {
                while (!queuedStates.isEmpty() && !wantStop.get()) {
                    SmartDashboard.putNumber("StateMachine/state", state.get());
                    currentState = queuedStates.poll();
                    currentState.onStart();
                    while (!currentState.isFinished() && !wantStop.get()) {
                        t_start = Timer.getFPGATimestamp();
                        System.out.println(t_start);
                        currentState.onLoop();
                        Timer.delay(delay - (Timer.getFPGATimestamp() - t_start));
                    }
                    currentState.onStop();
                    state.getAndAdd(1);
                }
            }
            stateLock.set(false);
        }catch (Exception e){
            System.out.println(e.getMessage());
            state.set(-3);
            SmartDashboard.putNumber("StateMachine/state", state.get());
            stateLock.set(false);
        }
    };

    public static boolean runMachine(StateMachineDescriptor descriptor) {
        SmartDashboard.putNumber("StateMachine/state", -1);
        if(stateLock.get()) return false;
        stateLock.set(true);
        wantStop.set(false);
        queuedStates = descriptor.getStates();
        Thread thread = new Thread(Man);
        thread.start();
        return true;
    }

    public static boolean isRunning(){
        return stateLock.get();
    }

    public static void assertStop(){
        SmartDashboard.putNumber("StateMachine/state", -1);
        if(!wantStop.get()){
            wantStop.set(true);
            System.out.println("State Machine Halting");
        }
        
    }


}
