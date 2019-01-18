package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.lib.loops.ILooper;
import frc.robot.Constants;

public class Alien extends Subsystem {

    private static Alien m_Alien = new Alien();
    private TalonSRX AlienS;
    private PeriodicIO Periodic;

    public Alien() {
        AlienS = new TalonSRX(Constants.ALIEN_ID);
    }

    public static Alien getInstance() {
        return m_Alien;
    }

    @Override
    public void outputTelemetry() {

    }

    public void readPeriodicInputs() {
        Periodic.AlienTopLimit = AlienS.getSensorCollection().isFwdLimitSwitchClosed();
        Periodic.AlienBottomLimit = AlienS.getSensorCollection().isRevLimitSwitchClosed();
    }

    public void writePeriodicOutputs() {
        if(Periodic.AlienBottomLimit && Periodic.AlienQuest == -1){AlienS.set(ControlMode.PercentOutput,0);}
        else if(Periodic.AlienTopLimit && Periodic.AlienQuest == 1){AlienS.set(ControlMode.PercentOutput,0);}
        else{AlienS.set(ControlMode.PercentOutput, Periodic.AlienQuest);}}


    @Override
    public void stop() {
        Periodic.AlienQuest = 0;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {

    }

    public void setAlienState(double state) {
        Periodic.AlienQuest = state;
    }

    public double GetAlienState() {
        return Periodic.AlienQuest;
    }

    @Override
    public void reset() {
        Periodic.AlienQuest = 0;
    }

    public class PeriodicIO {
        public double AlienQuest = 0;
        public boolean AlienTopLimit = false;
        public boolean AlienBottomLimit = false;
    }
}