package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.Loop;
import frc.robot.Constants;


public class Arm extends Subsystem {

    private static final Arm m_Arm = new Arm();
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
    private TalonSRX armProx, armDist, armWrist;
    private PeriodicIO periodic;
    private ArmModes ArmMode = ArmModes.DirectControl;


    public Arm() {
        armProx = new TalonSRX(Constants.ARM_PROXIMINAL);
        armDist = new TalonSRX(Constants.ARM_DISTAL);
        armWrist = new TalonSRX(Constants.ARM_WRIST);
        periodic = new PeriodicIO();
        reset();
    }

    public static Arm getInstance() {
        return m_Arm;
    }

    //Get Inputs and Use them
    @Override
    public void readPeriodicInputs() {
        periodic.prox = armProx.getSelectedSensorPosition() + periodic.absoluteProx;
        periodic.dist = armDist.getSelectedSensorPosition() + periodic.absoluteDist;
        periodic.wrist = armWrist.getSelectedSensorPosition() + periodic.absoluteWrist;
        if (ArmMode == ArmModes.DirectControl) {
            periodic.armProxPower = (SmartDashboard.getNumber("DB/Slider 0", 2.5) - 2.5) / 10;
            periodic.armDistPower = (SmartDashboard.getNumber("DB/Slider 1", 2.5) - 2.5) / 10;
            periodic.armWristPower = (SmartDashboard.getNumber("DB/Slider 2", 2.5) - 2.5) / 10;
        } else if (ArmMode == ArmModes.PID) {
            periodic.armProxPower = (SmartDashboard.getNumber("DB/Slider 0", 2.5) * 1536);
            periodic.armDistPower = (SmartDashboard.getNumber("DB/Slider 1", 2.5) * 1536);
            periodic.armWristPower = (SmartDashboard.getNumber("DB/Slider 2", 2.5) * 1536);
        }

    }

    //Outputs
    @Override
    public void writePeriodicOutputs() {
        if (periodic.armmode == ArmModes.DirectControl) {
            if (SmartDashboard.getBoolean("DB/Button 0", false)) {
                armProx.set(ControlMode.PercentOutput, periodic.armProxPower);
            } else {
            }
            if (SmartDashboard.getBoolean("DB/Button 1", false)) {
                armDist.set(ControlMode.PercentOutput, periodic.armDistPower);
            } else {
            }
            if (SmartDashboard.getBoolean("DB/Button 2", false)) {
                armWrist.set(ControlMode.PercentOutput, periodic.armWristPower);
            } else {
            }

        } else if (periodic.armmode == ArmModes.PID) {
            //if (SmartDashboard.getBoolean("DB/Button 0", false))//
            armProx.set(ControlMode.Position, periodic.armProxPower);
            //if (SmartDashboard.getBoolean("DB/Button 1", false))//
            armDist.set(ControlMode.Position, periodic.armDistPower);
            //if (SmartDashboard.getBoolean("DB/Button 2", false))
            armWrist.set(ControlMode.Position, periodic.armWristPower);
            //armEnd.set(ControlMode.Position, periodic.armEndPower);

        }
    }

    @Override
    public void outputTelemetry() {

        SmartDashboard.putNumber("Arm/Prox Absolute", periodic.absoluteProx);
        SmartDashboard.putNumber("Arm/Dist Absolute", periodic.absoluteDist);
        SmartDashboard.putNumber("Arm/Wrist Absolute", periodic.absoluteWrist);
        SmartDashboard.putNumber("Arm/Proximal Arm Power", periodic.armProxPower);
        SmartDashboard.putNumber("Arm/Distal Arm Power", periodic.armDistPower);
        SmartDashboard.putNumber("Arm/Wrist Arm Power", periodic.armWristPower);
        SmartDashboard.putNumber("Arm/Proximal Arm Error", armProx.getClosedLoopError());
        SmartDashboard.putNumber("Arm/Distal Arm Error", armDist.getClosedLoopError());
        SmartDashboard.putNumber("Arm/Proximal Arm Error", armWrist.getClosedLoopError());

        //SmartDashboard.putNumber("End Arm Power", periodic.armEndPower);
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
        periodic.absoluteDist = armDist.getSelectedSensorPosition();
        armDist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        armWrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        periodic.absoluteWrist = armWrist.getSelectedSensorPosition();
        armWrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        armProx.setSensorPhase(true); //TODO Find Sensor Phase for all talons
        armDist.setSensorPhase(true);
        armWrist.setSensorPhase(true);
        armProx.selectProfileSlot(0, 0);//TODO tune all PIDs
        armProx.config_kF(0, Constants.ARM_PROX_KF, 0);
        armProx.config_kP(0, Constants.ARM_PROX_KP, 0);
        armProx.config_kI(0, Constants.ARM_PROX_KI, 0);
        armProx.config_kD(0, Constants.ARM_PROX_KD, 0);
        armProx.config_IntegralZone(0, 0, 0);
        armDist.selectProfileSlot(0, 0);
        armDist.config_kF(0, Constants.ARM_DIST_KF, 0);
        armDist.config_kP(0, Constants.ARM_DIST_KP, 0);
        armDist.config_kI(0, Constants.ARM_DIST_KI, 0);
        armDist.config_kD(0, Constants.ARM_DIST_KD, 0);
        armDist.config_IntegralZone(0, 0, 0);
        armWrist.selectProfileSlot(0, 0);
        armWrist.config_kF(0, Constants.ARM_WRIST_KF, 0);
        armWrist.config_kP(0, Constants.ARM_WRIST_KP, 0);
        armWrist.config_kI(0, Constants.ARM_WRIST_KI, 0);
        armWrist.config_kD(0, Constants.ARM_WRIST_KD, 0);
        armWrist.config_IntegralZone(0, 0, 0);
        armProx.setNeutralMode(NeutralMode.Brake);
        armDist.setNeutralMode(NeutralMode.Brake);
        armWrist.setNeutralMode(NeutralMode.Brake);
        armProx.enableVoltageCompensation(true);
        armDist.enableVoltageCompensation(true);
        armWrist.enableVoltageCompensation(true);
        armProx.configVoltageCompSaturation(10);
        armDist.configVoltageCompSaturation(10);
        armWrist.configVoltageCompSaturation(10);
    }

    public void setArmProxPower (double Power)
    {
        periodic.armProxPower = Power;
    }

    public void setArmDistPower (double Power)
    {
        periodic.armDistPower = Power;
    }

    public void setArmWristPower (double Power)
    {
        periodic.armWristPower = Power;
    }



    public enum ArmModes {
        DirectControl,
        PID,
        StateSpace

    }

    public class PeriodicIO {
        //TALON POWERS
        double armProxPower = 0;
        double armDistPower = 0;
        double armWristPower = 0;
        //->||\TALON ANGLES ABSOLUTE
        double absoluteProx = 0;
        double absoluteDist = 0;
        double absoluteWrist = 0;
        //TALON ANGLE TARES
        double prox = 0;
        double dist = 0;
        double wrist = 0;

        ArmModes armmode = ArmModes.PID;
    }
}


