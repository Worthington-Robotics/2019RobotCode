package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.Loop;
import frc.robot.Constants;


public class Arm extends Subsystem {

    //Standard Declarations
    private static final Arm m_Arm = new Arm();
    private TalonSRX armProx, armDist, armWrist, armEnd;  // TODO Edit to make it 2 Prox 1 Dist 1 End (Provided that's ok)
    private PeriodicIO periodic;
    private ArmModes ArmMode = ArmModes.DirectControl;

    public static Arm getInstance() {
        return m_Arm;
    }

    //Declarations and such so things work (New thing)
    public Arm() {
        armProx = new TalonSRX(Constants.ARM_PROXIMINAL); // TODO See TODO above, Fix Accordingly
        armDist = new TalonSRX(Constants.ARM_DISTAL);
        armWrist = new TalonSRX(Constants.ARM_WRIST);
        armEnd = new TalonSRX(Constants.ARM_END);
        periodic = new PeriodicIO();
        reset();

    }

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

    //Get Inputs and Use them
    @Override
    public void readPeriodicInputs() {
        periodic.relativeProx = armProx.getSelectedSensorPosition();
        periodic.absoluteProx = periodic.absoluteProx + periodic.relativeProx;
        periodic.relativeDist = armDist.getSelectedSensorPosition();
        periodic.absoluteDist = periodic.absoluteDist + periodic.relativeDist;
        periodic.relativeWrist = armWrist.getSelectedSensorPosition();
        periodic.absoluteWrist = periodic.absoluteWrist + periodic.relativeWrist;
        periodic.relativeEnd = armEnd.getSelectedSensorPosition();
        periodic.absoluteEnd = periodic.absoluteEnd + periodic.relativeEnd;
        if (ArmMode == ArmModes.DirectControl) {
            periodic.armProxPower = (SmartDashboard.getNumber("DB/Slider 0", 2.5) - 2.5) / 2.5;
            periodic.armDistPower = (SmartDashboard.getNumber("DB/Slider 1", 2.5) - 2.5) / 2.5;
            periodic.armWristPower = (SmartDashboard.getNumber("DB/Slider 2", 2.5) - 2.5) / 2.5;
            periodic.armEndPower = (SmartDashboard.getNumber("DB/Slider 3", 2.5) - 2.5) / 2.5;
        } else if (ArmMode == ArmModes.PID) {
            periodic.armProxPower = (SmartDashboard.getNumber("DB/Slider 0", 2.5) - 2.5) * 1024;
            periodic.armDistPower = (SmartDashboard.getNumber("DB/Slider 1", 2.5) - 2.5) * 1536;
            periodic.armWristPower = (SmartDashboard.getNumber("DB/Slider 2", 2.5) - 2.5) * 1536;
            periodic.armEndPower = (SmartDashboard.getNumber("DB/Slider 3", 2.5) - 2.5) * 1536;
        }

    }

    //Outputs
    @Override
    public void writePeriodicOutputs() {
        if (ArmMode == ArmModes.DirectControl) {
            armProx.set(ControlMode.PercentOutput, periodic.armProxPower);
            armDist.set(ControlMode.PercentOutput, periodic.armDistPower);
            armWrist.set(ControlMode.PercentOutput, periodic.armWristPower);
            armEnd.set(ControlMode.PercentOutput, periodic.armEndPower);
        } else if (ArmMode == ArmModes.PID) {
            armProx.set(ControlMode.Position, periodic.armProxPower);
            armDist.set(ControlMode.Position, periodic.armDistPower);
            armWrist.set(ControlMode.Position, periodic.armWristPower);
            armEnd.set(ControlMode.Position, periodic.armEndPower);

        }
    }

    //Driver Station Things for Monitoring
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
        armProx.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        periodic.absoluteProx = armProx.getSelectedSensorPosition();
        armProx.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        armDist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        periodic.absoluteDist = armProx.getSelectedSensorPosition();
        armDist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        armWrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        periodic.absoluteWrist = armProx.getSelectedSensorPosition();
        armWrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        armEnd.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        periodic.absoluteEnd = armProx.getSelectedSensorPosition();
        armEnd.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        armProx.setSensorPhase(false); //TODO Find Sensor Phase for all talons
        armDist.setSensorPhase(true);
        armWrist.setSensorPhase(true);
        armProx.selectProfileSlot(0, 0);//TODO tune all PIDs
        armProx.config_kF(0, 0, 0);
        armProx.config_kP(0, .25, 0);
        armProx.config_kI(0, 0, 0);
        armProx.config_kD(0, 0, 0);
        armProx.config_IntegralZone(0, 0, 0);
        armDist.selectProfileSlot(0, 0);
        armDist.config_kF(0, 0, 0);
        armDist.config_kP(0, .25, 0);
        armDist.config_kI(0, 0, 0);
        armDist.config_kD(0, 0, 0);
        armDist.config_IntegralZone(0, 0, 0);
        armWrist.selectProfileSlot(0, 0);
        armWrist.config_kF(0, 0, 0);
        armWrist.config_kP(0, .25, 0);
        armWrist.config_kI(0, 0, 0);
        armWrist.config_kD(0, 0, 0);
        armWrist.config_IntegralZone(0, 0, 0);
    }

    public class PeriodicIO {
        public double armProxPower = 0;
        public double armDistPower = 0;
        public double armWristPower = 0;
        public double armEndPower = 0;
        public double absoluteProx = 0;
        public double absoluteDist = 0;
        public double absoluteWrist = 0;
        public double absoluteEnd = 0;
        public double relativeProx = 0;
        public double relativeDist = 0;
        public double relativeWrist = 0;
        public double relativeEnd = 0;
    }

    public enum ArmModes {
        DirectControl,
        PID,
        StateSpace

    }

    public double convertEncoderCounts(int counts)
    {
        double radians = counts;
        radians *= 2*Math.PI;
        radians /= 4096;
        return radians;

    }
}


