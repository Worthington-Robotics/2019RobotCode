package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private TalonSRX armProx, armDist;
    private PeriodicIO periodic;
    private Ultrasonic US1, US2;

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
        periodic.proxRel = armProx.getSensorCollection().getQuadraturePosition();
        periodic.distRel = armDist.getSensorCollection().getQuadraturePosition();
        periodic.distAbsolute = armDist.getSelectedSensorPosition();
        periodic.proxAbsolute = armProx.getSelectedSensorPosition();

        periodic.enableProx = SmartDashboard.getBoolean("DB/Button 0", false);
        periodic.enableDist = SmartDashboard.getBoolean("DB/Button 1", false);


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
                break;
            case PID:
                //System.out.println("pid update");
                armProx.set(ControlMode.Position, periodic.armProxPower);
                armDist.set(ControlMode.Position, periodic.armDistPower);
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
        SmartDashboard.putNumber("Arm/Prox Rel",periodic.proxRel);
        //
        SmartDashboard.putNumber("Arm/Dist Mod", periodic.distMod);
        SmartDashboard.putNumber("Arm/Dist Absolute", periodic.distAbsolute);
        SmartDashboard.putNumber("Arm/Distal Arm Power", periodic.armDistPower);
        SmartDashboard.putNumber("Arm/Distal Arm Error", periodic.distError);
        SmartDashboard.putNumber("Arm/Dist Rel",periodic.distRel);
        //
        SmartDashboard.putString("Arm/Mode", periodic.armmode.toString());
    }


    public void reset() {
        periodic = new PeriodicIO();
        resetArmMod();
        configTalons();
    }

    public void resetArmMod() {
        periodic.proxMod = Constants.ProxAbsoluteZero - periodic.proxAbsolute;
        periodic.distMod = Constants.DistAbsoluteZero - periodic.distAbsolute;
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
        armProx.setSensorPhase(false);
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
        armDist.setSensorPhase(false);
    }

    public void setPIDArmConfig(ArmStates state) {
        if (periodic.armmode != ArmModes.PID) {
            periodic.armmode = ArmModes.PID;
        }
        periodic.armProxPower = state.prox;
        periodic.armDistPower = state.dist;
        /*periodic.armProxPower = config.proximal - periodic.proxMod;
        periodic.armDistPower = config.distal - periodic.distMod;*/
    }

    public void setSSArmConfig(ArmStates state) {
        if (periodic.armmode != ArmModes.STATE_SPACE) {
            periodic.armmode = ArmModes.STATE_SPACE;
        }
        periodic.armProxPower = state.prox;
        periodic.armDistPower = state.dist;
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
        STATE_SPACE;
        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    public class PeriodicIO {
        //joint enable booleans
        boolean enableProx = true;
        boolean enableDist = true;
        //TALON POWERS
        double armProxPower = 0;
        double armDistPower = 0;
        //->||\TALON ANGLES ABSOLUTE
        double proxAbsolute = armProx.getSelectedSensorPosition();
        double distAbsolute = armDist.getSelectedSensorPosition();
        //TALON MODS
        double proxMod = 0;
        double distMod = 0;
        //TALON ERROR
        double proxError = armProx.getClosedLoopError();
        double distError = armDist.getClosedLoopError();
        //PRIOR DISTANCE
        double US1Past = 0;
        double US2Past = 0;
        double US1Dis = 0;
        double US2Dis = 0;
        //TALON RELS
        double proxRel = 0;
        double distRel = 0;

        ArmModes armmode = ArmModes.PID;
    }

    public enum ArmStates {
        FWD_GROUND_CARGO(-2000, -2500),
        FWD_LOW_HATCH(0, 0),
        FWD_LOW_CARGO(2048, -1536),
        FWD_MEDIUM_HATCH(2048, 0),
        FWD_MEDIUM_CARGO(1024, -1024),
        FWD_HIGH_HATCH(0, 512),
        FWD_HIGH_CARGO(2560, -1536),

        REV_MEDIUM_HATCH(0, 0),
        REV_MEDIUM_CARGO(0, 0),
        REV_HIGH_HATCH(0, 0),
        REV_HIGH_CARGO(0, 0),
        REV_GROUND_CARGO(0, 0),

        GROUND_HATCH(0, 0),
        STOW_ARM(512, 512);

        private double prox, dist;

        ArmStates(double prox, double dist) {
            this.prox = prox;
            this.dist = dist;
        }

        public double getProx() {
            return prox;
        }

        public double getDist() {
            return dist;
        }
    }

}


