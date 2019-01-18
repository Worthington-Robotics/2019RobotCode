package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import frc.lib.loops.ILooper;
import frc.robot.Constants;

public class Alien extends Subsystem {

    private static Alien m_Alien = new Alien();
    private TalonSRX AlienS;
   private PeriodicIO Periodic;
   private Alien m_Instance;
    public Alien() {
        AlienS = new TalonSRX(Constants.ALIEN_ID);
    }

    public Alien getm_Instance() {
        return m_Instance;
    }

    @Override
    public void outputTelemetry() {

    }

    public void writePeriodicOutputs() {
        AlienS.set(ControlMode.PercentOutput, Periodic.AlienQuest);
    }

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
    public class PeriodicIO{public double AlienQuest = 0;}
}