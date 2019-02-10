package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.Loop;
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

    private final Loop aloop = new Loop() {


        public void onStart(double timestamp) {

        }


        public void onLoop(double timestamp) {

        }


        public void onStop(double timestamp) {

        }
    };

    public Arm() {
        armProx = new TalonSRX(Constants.ARM_PRONOMINAL);
        armDist = new TalonSRX(Constants.ARM_DISTAL);
        armWrist = new TalonSRX(Constants.ARM_WRIST);
        reset();
    }

    public void readPeriodicInputs() {
        periodic.proxError = armProx.getClosedLoopError();
        periodic.distError = armDist.getClosedLoopError();
        periodic.wristError = armWrist.getClosedLoopError();

        periodic.enableProx = SmartDashboard.getBoolean("DB/Button 0", false);
        periodic.enableDist = SmartDashboard.getBoolean("DB/Button 1", false);
        periodic.enableWrist = SmartDashboard.getBoolean("DB/Button 2", false);

        if (ArmMode == ArmModes.DirectControl) {
            periodic.armProxPower = (SmartDashboard.getNumber("DB/Slider 0", 2.5) - 2.5) / 5;
            periodic.armDistPower = (SmartDashboard.getNumber("DB/Slider 1", 2.5) - 2.5) / 5;
            periodic.armWristPower = (SmartDashboard.getNumber("DB/Slider 2", 2.5) - 2.5) / 5;
        } else if (ArmMode == ArmModes.PID) {
            periodic.armProxPower = (SmartDashboard.getNumber("DB/Slider 0", 2.5) * 1536);
            periodic.armDistPower = (SmartDashboard.getNumber("DB/Slider 1", 2.5) * 1536);
            periodic.armWristPower = (SmartDashboard.getNumber("DB/Slider 2", 2.5) * 1536);
        }

    }

    public void writePeriodicOutputs() {
        switch(periodic.armmode){
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
                armProx.set(ControlMode.Position, periodic.armProxPower);
                armDist.set(ControlMode.Position, periodic.armDistPower);
                armWrist.set(ControlMode.Position, periodic.armWristPower);
                break;
            case STATE_SPACE:

                break;
            default:
                System.out.println("Arm entered unexpected control state!");
        }
    }

    @Override
    public void outputTelemetry() {
        //TODO REMOVE ALL SENSOR CALLS FROM HERE -- bad phillip
        //Literally breaks design pattern
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

    public void stop() {

    }

    @Override
    public void reset() {
        periodic = new PeriodicIO();
        resetArmMod();
        configTalons();
    }

    public void resetArmMod()
    {
        double PStartConfig = armProx.getSensorCollection().getPulseWidthPosition();
        double DStartConfig = armDist.getSensorCollection().getPulseWidthPosition();
        double WStartConfig = armWrist.getSensorCollection().getPulseWidthPosition();
        periodic.proxMod = Constants.ProxAbsoluteZero - PStartConfig;
        periodic.distMod = Constants.DistAbsoluteZero - DStartConfig;
        periodic.wristMod = Constants.WristAbsoluteZero - WStartConfig;
    }
    public void configTalons(){
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

    public void setPIDArmConfig(ArmConfiguration config){
        if(periodic.armmode != ArmModes.PID){
            periodic.armmode = ArmModes.PID;
        }
        periodic.armProxPower = config.proximal;
        periodic.armDistPower = config.distal;
        periodic.armWristPower = config.wrist;
    }

    public void setSSArmConfig(ArmConfiguration config){
        if(periodic.armmode != ArmModes.STATE_SPACE){
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

        ArmModes armmode = ArmModes.DirectControl;
    }

    public static class ArmConfiguration{

        double proximal, distal, wrist;

        public ArmConfiguration(double proxAngle, double distAngle, double wristAngle){
            proximal = proxAngle;
            distal = distAngle;
            wrist = wristAngle;
        }
    }
}


