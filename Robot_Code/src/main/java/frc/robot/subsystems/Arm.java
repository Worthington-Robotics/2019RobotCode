package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.Loop;
import frc.lib.util.Ultrasonic;
import frc.robot.Constants;

/**
 * The code for governing the three-axis arm
 * using a set of position PIDs with absolute and relative MagEncoder Values
 */
public class Arm extends Subsystem {

    private static final Arm m_Arm = new Arm();

    /**
     * @return the only arm instance
     */
    public static Arm getInstance() {
        return m_Arm;
    }

    private TalonSRX armProx, armDist, armWrist;
    private PeriodicIO periodic;
    private ArmModes ArmMode = ArmModes.DirectControl;
    private Ultrasonic US1, US2;

    private final Loop aloop = new Loop() {


        public void onStart(double timestamp) {

        }


        public void onLoop(double timestamp) {

        }


        public void onStop(double timestamp) {

        }
    };

    private Arm() {
        armProx = new TalonSRX(Constants.ARM_PRONOMINAL);
        armDist = new TalonSRX(Constants.ARM_DISTAL);
        US1 = new Ultrasonic(Constants.ULTRASONIC_IN_1, Constants.ULTRASONIC_OUT_1);
        US2 = new Ultrasonic(Constants.ULTRASONIC_IN_2, Constants.ULTRASONIC_OUT_2);
        reset();
    }

    public void readPeriodicInputs() {
        periodic.US1Past = periodic.US1Dis;
        periodic.US2Past = periodic.US2Dis;
        periodic.US1Dis = US1.getDistance();
        periodic.US2Dis = US2.getDistance();
        periodic.proxError = armProx.getClosedLoopError();
        periodic.distError = armDist.getClosedLoopError();
        periodic.wristError = armWrist.getClosedLoopError();
        periodic.proxRel = armProx.getSensorCollection().getQuadraturePosition();
        periodic.distRel = armDist.getSensorCollection().getQuadraturePosition();
        periodic.wristRel = armWrist.getSensorCollection().getQuadraturePosition();

        periodic.enableProx = SmartDashboard.getBoolean("DB/Button 0", false);
        periodic.enableDist = SmartDashboard.getBoolean("DB/Button 1", false);
        periodic.enableWrist = SmartDashboard.getBoolean("DB/Button 2", false);

        if (ArmMode == ArmModes.DirectControl) {
            periodic.armProxPower = (SmartDashboard.getNumber("DB/Slider 0", 2.5) - 2.5) / 5;
            periodic.armDistPower = (SmartDashboard.getNumber("DB/Slider 1", 2.5) - 2.5) / 5;
            periodic.armWristPower = (SmartDashboard.getNumber("DB/Slider 2", 2.5) - 2.5) / 5;
        } else if (ArmMode == ArmModes.PID) {
            periodic.armProxPower = (SmartDashboard.getNumber("DB/Slider 0", 2.5) * Constants.DRIVE_ENCODER_PPR / 4 * 3 - periodic.proxMod);
            periodic.armDistPower = (SmartDashboard.getNumber("DB/Slider 1", 2.5) * Constants.DRIVE_ENCODER_PPR / 4 * 3 - periodic.distMod);
            periodic.armWristPower = (SmartDashboard.getNumber("DB/Slider 2", 2.5) * Constants.DRIVE_ENCODER_PPR / 4 * 3 - periodic.wristMod);
        }

    }

    public void writePeriodicOutputs() {
        switch (periodic.armmode) {
            case DirectControl:
                if (periodic.enableProx) {
                    armProx.set(ControlMode.PercentOutput, periodic.armProxPower);
                }
                if (periodic.enableDist) {
                    armDist.set(ControlMode.PercentOutput, periodic.armDistPower);
                }
                if (periodic.enableWrist) {
                    armWrist.set(ControlMode.PercentOutput, periodic.armWristPower);
                }
                break;
            case PID:
                armProx.set(ControlMode.Position, periodic.armProxPower, DemandType.ArbitraryFeedForward, Math.cos(periodic.proxRel - periodic.proxMod / 4096 * 2 * Math.PI) * Constants.ARM_PROX_A_FEEDFORWARD);
                armDist.set(ControlMode.Position, periodic.armDistPower, DemandType.ArbitraryFeedForward, Math.cos(periodic.distRel - periodic.distMod / 4096 * 2 * Math.PI) * Constants.ARM_DIST_A_FEEDFORWARD);
                armWrist.set(ControlMode.Position, periodic.armWristPower, DemandType.ArbitraryFeedForward, Math.cos(periodic.wristRel - periodic.wristMod / 4096 * 2 * Math.PI) * Constants.ARM_WRIST_A_FEEDFORWARD);
                break;
            case STATE_SPACE:

                break;
            default:
                System.out.println("Arm entered unexpected control state!");
        }
    }

    public void outputTelemetry() {
        SmartDashboard.putNumber("Arm/Prox Mod", periodic.proxMod);
        SmartDashboard.putNumber("Arm/Prox Absolute", periodic.proxAbsolute);
        SmartDashboard.putNumber("Arm/Proximal Arm Power", periodic.armProxPower);
        SmartDashboard.putNumber("Arm/Proximal Arm Error", periodic.proxError);
        //
        SmartDashboard.putNumber("Arm/Dist Mod", periodic.distMod);
        SmartDashboard.putNumber("Arm/Dist Absolute", periodic.distAbsolute);
        SmartDashboard.putNumber("Arm/Distal Arm Power", periodic.armDistPower);
        SmartDashboard.putNumber("Arm/Distal Arm Error", periodic.distError);
        //
        SmartDashboard.putNumber("Arm/Wrist Mod", periodic.wristMod);
        SmartDashboard.putNumber("Arm/Wrist Absolute", periodic.wristAbsolute);
        SmartDashboard.putNumber("Arm/Wrist Arm Power", periodic.armWristPower);
        SmartDashboard.putNumber("Arm/Wrist Arm Error", periodic.wristError);

    }


    public void reset() {
        periodic = new PeriodicIO();
        resetArmMod();
        configTalons();
    }

    public void resetArmMod() {
        periodic.proxMod = Constants.ProxAbsoluteZero - periodic.proxAbsolute;
        periodic.distMod = Constants.DistAbsoluteZero - periodic.distAbsolute;
        periodic.wristMod = Constants.WristAbsoluteZero - periodic.wristAbsolute;
    }

    public void configTalons() {
        armProx.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        armProx.setSensorPhase(true);
        armProx.selectProfileSlot(0, 0);//TODO tune all PIDs
        armProx.config_kF(0, Constants.ARM_PROX_KF, 0);
        armProx.config_kP(0, Constants.ARM_PROX_KP, 0);
        armProx.config_kI(0, Constants.ARM_PROX_KI, 0);
        armProx.config_kD(0, Constants.ARM_PROX_KD, 0);
        armProx.config_IntegralZone(0, 0, 0);
        armProx.setNeutralMode(NeutralMode.Brake);
        armProx.configVoltageCompSaturation(10);
        armProx.enableVoltageCompensation(true);
        //
        armDist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        armDist.setSensorPhase(true);
        armDist.selectProfileSlot(0, 0);
        armDist.config_kF(0, Constants.ARM_DIST_KF, 0);
        armDist.config_kP(0, Constants.ARM_DIST_KP, 0);
        armDist.config_kI(0, Constants.ARM_DIST_KI, 0);
        armDist.config_kD(0, Constants.ARM_DIST_KD, 0);
        armDist.config_IntegralZone(0, 0, 0);
        armDist.setNeutralMode(NeutralMode.Brake);
        armDist.configVoltageCompSaturation(10);
        armDist.enableVoltageCompensation(true);
        //
        armWrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        armWrist.setSensorPhase(true);
        armWrist.selectProfileSlot(0, 0);
        armWrist.config_kF(0, Constants.ARM_WRIST_KF, 0);
        armWrist.config_kP(0, Constants.ARM_WRIST_KP, 0);
        armWrist.config_kI(0, Constants.ARM_WRIST_KI, 0);
        armWrist.config_kD(0, Constants.ARM_WRIST_KD, 0);
        armWrist.config_IntegralZone(0, 0, 0);
        armWrist.setNeutralMode(NeutralMode.Brake);
        armWrist.configVoltageCompSaturation(10);
        armWrist.enableVoltageCompensation(true);

    }

    public void setPIDArmConfig(ArmConfiguration config) {
        if (periodic.armmode != ArmModes.PID) {
            periodic.armmode = ArmModes.PID;
        }
        periodic.armProxPower = config.proximal * Constants.DRIVE_ENCODER_PPR * 3 / 4;
        periodic.armDistPower = config.distal * Constants.DRIVE_ENCODER_PPR * 3 / 4;
        periodic.armWristPower = config.wrist * Constants.DRIVE_ENCODER_PPR * 3 / 4;
    }

    public void setSSArmConfig(ArmConfiguration config) {
        if (periodic.armmode != ArmModes.STATE_SPACE) {
            periodic.armmode = ArmModes.STATE_SPACE;
        }
        periodic.armProxPower = config.proximal;
        periodic.armDistPower = config.distal;
        periodic.armWristPower = config.wrist;
    }

    public void setArmProxPower(double Power) {
        periodic.armProxPower = Power;
    }

    public void setArmDistPower(double Power) {
        periodic.armDistPower = Power;
    }

    public void setArmWristPower(double Power) {
        periodic.armWristPower = Power;
    }

    public double getUltrasonicDistance() {

        if ((periodic.US1Dis - periodic.US1Past > -Constants.US_UPDATE_RATE && periodic.US1Dis - periodic.US1Past < Constants.US_UPDATE_RATE)
                && (periodic.US2Dis - periodic.US2Past > -Constants.US_UPDATE_RATE && periodic.US2Dis - periodic.US2Past < Constants.US_UPDATE_RATE) &&
                (periodic.US1Dis > Constants.US_SENSOR_OFFSET && periodic.US2Dis > Constants.US_SENSOR_OFFSET)) {
            return (periodic.US1Dis + periodic.US2Dis) / 2;
        } else if ((periodic.US1Dis - periodic.US1Past > -Constants.US_UPDATE_RATE && periodic.US1Dis - periodic.US1Past < Constants.US_UPDATE_RATE)
                && (periodic.US2Dis - periodic.US2Past > -Constants.US_UPDATE_RATE && periodic.US2Dis - periodic.US2Past < Constants.US_UPDATE_RATE) ||
                (periodic.US1Dis < Constants.US_SENSOR_OFFSET && periodic.US2Dis > Constants.US_SENSOR_OFFSET)) {
            return periodic.US2Dis;
        } else {

            return periodic.US1Dis;
        }
    }


    public enum ArmModes {
        DirectControl,
        PID,
        STATE_SPACE

    }

    public class PeriodicIO {
        //joint enable booleans
        boolean enableProx = true;
        boolean enableDist = true;
        boolean enableWrist = true;
        //TALON POWERS
        double armProxPower = 0;
        double armDistPower = 0;
        double armWristPower = 0;
        //->||\TALON ANGLES ABSOLUTE
        double proxAbsolute = armProx.getSensorCollection().getPulseWidthPosition();
        double distAbsolute = armDist.getSensorCollection().getPulseWidthPosition();
        double wristAbsolute = armWrist.getSensorCollection().getPulseWidthPosition();
        //TALON MODS
        double proxMod = 0;
        double distMod = 0;
        double wristMod = 0;
        //TALON ERROR
        double proxError = armProx.getClosedLoopError();
        double distError = armDist.getClosedLoopError();
        double wristError = armWrist.getClosedLoopError();
        //PRIOR DISTANCE
        double US1Past = 0;
        double US2Past = 0;
        double US1Dis = 0;
        double US2Dis = 0;
        //TALON RELS
        double proxRel = 0;
        double distRel = 0;
        double wristRel = 0;

        ArmModes armmode = ArmModes.DirectControl;
    }

    public static class ArmConfiguration {

        double proximal, distal, wrist;

        public ArmConfiguration(double proxAngle, double distAngle, double wristAngle) {
            //in rotations
            proximal = proxAngle;
            distal = distAngle;
            wrist = wristAngle;
        }
    }

}


