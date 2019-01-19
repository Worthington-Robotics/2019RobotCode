    package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.Loop;
import frc.robot.Constants;


public class Arm extends Subsystem {

    private static final Arm m_Arm = new Arm();
    private TalonSRX armProx, armDist, armWrist, armEnd;
    private PeriodicIO periodic;
    public static Arm getInstance() {return m_Arm;}
    public Arm() {
        armProx = new TalonSRX(Constants.ARM_PROXIMINAL);
        armDist = new TalonSRX(Constants.ARM_DISTAL);
        armWrist = new TalonSRX(Constants.ARM_WRIST);
        armEnd = new TalonSRX(Constants.ARM_END);
        periodic = new PeriodicIO();
    }
    //LOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP
    private final Loop aloop = new Loop() {
        @Override
        public void onStart(double timestamp) {

        }
        @Override
        public void onLoop(double timestamp) {

        }

        @Override
        public void onStop(double timestamp) {

        }
    };

    @Override
    public void readPeriodicInputs() {
        periodic.armProxPower = SmartDashboard.getNumber("DB/Slider 0", 2.5);
        periodic.armDistPower = SmartDashboard.getNumber("DB/Slider 1", 2.5);
        periodic.armWristPower = SmartDashboard.getNumber("DB/Slider 2",2.5);
        periodic.armEndPower = SmartDashboard.getNumber("DB/Slider 3", 2.5);
    }

    @Override
    public void writePeriodicOutputs() {
        armProx.set(ControlMode.PercentOutput, periodic.armProxPower);
        armDist.set(ControlMode.PercentOutput, periodic.armDistPower);
        armWrist.set(ControlMode.PercentOutput, periodic.armWristPower);
        armEnd.set(ControlMode.PercentOutput, periodic.armEndPower);
    }
    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Proximal Arm Power", periodic.armProxPower);
        SmartDashboard.putNumber("Distal Arm Power", periodic.armDistPower);
        SmartDashboard.putNumber("Wrist Arm Power", periodic.armWristPower);
        SmartDashboard.putNumber("End Arm Power", periodic.armEndPower);
    }

    @Override
    public void stop() {

    }

    @Override
    public void reset() {
        periodic = new PeriodicIO();
    }

    public class PeriodicIO {
        public double armProxPower = 0;
        public double armDistPower = 0;
        public double armWristPower = 0;
        public double armEndPower = 0;
    }
}


