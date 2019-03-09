package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.HIDHelper;
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
    //private Ultrasonic US1, US2;
    private double[] operatorInput;

    private Arm() {
        armProx = new TalonSRX(Constants.ARM_PRONOMINAL);
        armDist = new TalonSRX(Constants.ARM_DISTAL);
        armProx.configContinuousCurrentLimit(20);
        armDist.configContinuousCurrentLimit(20);
        armDist.enableCurrentLimit(true);
        armProx.enableCurrentLimit(true);
        //US1 = new Ultrasonic(Constants.ULTRASONIC_IN_1, Constants.ULTRASONIC_OUT_1);
        //US2 = new Ultrasonic(Constants.ULTRASONIC_IN_2, Constants.ULTRASONIC_OUT_2);
        reset();
    }

    public void readPeriodicInputs() {
        periodic.operatorInput = HIDHelper.getAdjStick(Constants.LAUNCHPAD_STICK);
        periodic.sideShift = Constants.LAUNCH_PAD.getRawButton(9);
        periodic.proxPrev = periodic.proxRel;
        periodic.distPrev = periodic.distRel;
        periodic.US1Past = periodic.US1Dis;
        periodic.US2Past = periodic.US2Dis;
        //periodic.US1Dis = US1.getDistance();
        //periodic.US2Dis = US2.getDistance();
        periodic.proxAmps = armProx.getOutputCurrent();
        periodic.distAmps = armDist.getOutputCurrent();
        periodic.proxError = armProx.getClosedLoopError();
        periodic.distError = armDist.getClosedLoopError();
        periodic.proxRel = armProx.getSensorCollection().getQuadraturePosition();
        periodic.distRel = armDist.getSensorCollection().getQuadraturePosition();
        periodic.distAbsolute = armDist.getSensorCollection().getPulseWidthPosition();
        periodic.proxAbsolute = armProx.getSensorCollection().getPulseWidthPosition();

        periodic.enableProx = SmartDashboard.getBoolean("DB/Button 0", false);
        periodic.enableDist = SmartDashboard.getBoolean("DB/Button 1", false);
        if (periodic.armmode == ArmModes.DirectControl) {
            if (periodic.enableProx) {
                periodic.armProxPower = (SmartDashboard.getNumber("DB/Slider 0", 2.5) - 2.5) / 2.5;
            }
            if (periodic.enableDist) {
                periodic.armDistPower = (SmartDashboard.getNumber("DB/Slider 1", 2.5) - 2.5) / 2.5;
            }
        }


    }


    public void writePeriodicOutputs() {
        switch (periodic.armmode) {
            case DirectControl:
                /*if((Math.sin((periodic.armProxPower + periodic.proxMod) / 2048 * Math.PI)*27)+(Math.sin((periodic.armDistPower + periodic.distMod) / 2048 * Math.PI)*21) == -28)
                    periodic.armDistPower = -.25;*/
                armProx.set(ControlMode.PercentOutput, periodic.operatorInput[0]);
                armDist.set(ControlMode.PercentOutput, periodic.operatorInput[1]);
                break;
            case PID:
                armProx.set(ControlMode.Position, periodic.armProxPower + periodic.proxMod, DemandType.ArbitraryFeedForward, Constants.ARM_PROX_A_FEEDFORWARD * Math.sin((periodic.armProxPower + periodic.proxMod) / 2048 * Math.PI));
                armDist.set(ControlMode.Position, periodic.armDistPower + periodic.distMod/*, DemandType.ArbitraryFeedForward, Math.sin(periodic.armDistPower + periodic.distMod / 2048 * Math.PI)*/);
                break;
            case STATE_SPACE:

                break;
            case SAFETY_CATCH:
                armProx.set(ControlMode.PercentOutput, 0);
                armDist.set(ControlMode.PercentOutput, 0);
                break;
            default:
                System.out.println("Arm entered unexpected control state!");
        }
    }

    public void outputTelemetry() {
        SmartDashboard.putNumber("Arm/Prox Mod", periodic.proxMod);
        SmartDashboard.putNumber("Arm/Proximal Arm Power", periodic.armProxPower);
        SmartDashboard.putNumber("Arm/Proximal Arm Error", periodic.proxError);
        SmartDashboard.putNumber("Arm/Prox Rel", periodic.proxRel);
        SmartDashboard.putNumber("Arm/Prox Point", periodic.proxRel - periodic.proxMod);
        SmartDashboard.putNumber("Arm/Prox Dial", periodic.operatorInput[0]);
        SmartDashboard.putNumber("Arm/Prox Abs", periodic.proxAbsolute);
        //
        SmartDashboard.putNumber("Arm/Dist Mod", periodic.distMod);
        SmartDashboard.putNumber("Arm/Distal Arm Power", periodic.armDistPower);
        SmartDashboard.putNumber("Arm/Distal Arm Error", periodic.distError);
        SmartDashboard.putNumber("Arm/Dist Rel", periodic.distRel);
        SmartDashboard.putNumber("Arm/Dist Point", periodic.distRel - periodic.distMod);
        SmartDashboard.putNumber("Arm/Dist Dial", periodic.operatorInput[1]);
        SmartDashboard.putNumber("Arm/Dist Abs", periodic.distAbsolute);
        //
        SmartDashboard.putString("Arm/Mode", periodic.armmode.toString());
        SmartDashboard.putBoolean("Arm/Stowed", periodic.stowed);
    }


    public void reset() {
        periodic = new PeriodicIO();
        resetArmMod();
        configTalons();
    }

    public void resetArmMod() {
        periodic.proxMod = Constants.PROX_ABSOLUTE_ZERO - periodic.proxAbsolute;
        periodic.distMod = Constants.DIST_ABSOLUTE_ZERO - periodic.distAbsolute;
    }

    public void configTalons() {
        armProx.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        armProx.selectProfileSlot(0, 0);//TODO tune all PIDs
        armProx.config_kF(0, Constants.ARM_PROX_KF, 0);
        armProx.config_kP(0, Constants.ARM_PROX_KP, 0);
        armProx.config_kI(0, Constants.ARM_PROX_KI, 0);
        armProx.config_kD(0, Constants.ARM_PROX_KD, 0);
        armProx.config_IntegralZone(0, 0, 0);
        armProx.setNeutralMode(NeutralMode.Brake);
        armProx.configVoltageCompSaturation(10);
        armProx.enableVoltageCompensation(true);
        armProx.setInverted(true);
        armProx.setSensorPhase(true);
        armProx.setSelectedSensorPosition(0);
        //
        armDist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        armDist.selectProfileSlot(0, 0);
        armDist.config_kF(0, Constants.ARM_DIST_KF, 0);
        armDist.config_kP(0, Constants.ARM_DIST_KP, 0);
        armDist.config_kI(0, Constants.ARM_DIST_KI, 0);
        armDist.config_kD(0, Constants.ARM_DIST_KD, 0);
        armDist.config_IntegralZone(0, 0, 0);
        armDist.setNeutralMode(NeutralMode.Brake);
        armDist.configVoltageCompSaturation(10);
        armDist.enableVoltageCompensation(true);
        armDist.setInverted(false);
        armDist.setSensorPhase(false);
        armDist.setSelectedSensorPosition(0);
    }

    public void setPIDArmConfig(ArmStates state) {
        if (periodic.armmode != ArmModes.PID) {
            periodic.armmode = ArmModes.PID;
        }
        periodic.armProxPower = state.prox;
        periodic.armDistPower = state.dist;
    }

    public void setSSArmConfig(ArmStates state) {
        if (periodic.armmode != ArmModes.STATE_SPACE) {
            periodic.armmode = ArmModes.STATE_SPACE;
        }
        periodic.armProxPower = state.prox;
        periodic.armDistPower = state.dist;
    }

    public void safeMode() {
        periodic.armmode = ArmModes.SAFETY_CATCH;
    }

    public void setVelocitymConfig(double prox, double dist) {
        if (periodic.armmode != ArmModes.DirectControl) {
            periodic.armmode = ArmModes.DirectControl;
        }
        periodic.armProxPower = prox;
        periodic.armDistPower = dist;
    }

    public double getProxPoint() {
        return periodic.proxRel - periodic.proxMod;
    }

    public double getDistPoint() {
        return periodic.distRel - periodic.distMod;
    }

    //public double getUltrasonicDistance() {

        //if ((periodic.US1Dis - periodic.US1Past > -Constants.US_UPDATE_RATE && periodic.US1Dis - periodic.US1Past < Constants.US_UPDATE_RATE)
          //      && (periodic.US2Dis - periodic.US2Past > -Constants.US_UPDATE_RATE && periodic.US2Dis - periodic.US2Past < Constants.US_UPDATE_RATE) &&
            //    (periodic.US1Dis > Constants.US_SENSOR_OFFSET && periodic.US2Dis > Constants.US_SENSOR_OFFSET)) {
            //return (periodic.US1Dis + periodic.US2Dis) / 2;
        //} else if ((periodic.US1Dis - periodic.US1Past > -Constants.US_UPDATE_RATE && periodic.US1Dis - periodic.US1Past < Constants.US_UPDATE_RATE)
          //      && (periodic.US2Dis - periodic.US2Past > -Constants.US_UPDATE_RATE && periodic.US2Dis - periodic.US2Past < Constants.US_UPDATE_RATE) ||
            //    (periodic.US1Dis < Constants.US_SENSOR_OFFSET && periodic.US2Dis > Constants.US_SENSOR_OFFSET)) {
            //return periodic.US2Dis;
        //} else {
    public boolean getStowed()
    {
        return periodic.stowed;
    }

    public void setStowed(boolean stowed)
    {
        periodic.stowed = stowed;
    }

    public boolean getSideShift()
    {
        return periodic.sideShift;
    }

    public double[] getAbsolute()
    {
        return new double[]{periodic.proxAbsolute , periodic.distAbsolute};
    }





    public enum ArmModes {
        DirectControl,
        PID,
        STATE_SPACE,
        SAFETY_CATCH;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    public class PeriodicIO {
        //Side Shift
        boolean sideShift = false;
        //joint enable booleans
        boolean enableProx = false;
        boolean enableDist = false;
        //TALON POWERS
        double armProxPower = 0;
        double armDistPower = 0;
        //->||\TALON ANGLES ABSOLUTE
        double proxAbsolute = armProx.getSensorCollection().getPulseWidthPosition();
        double distAbsolute = armDist.getSensorCollection().getPulseWidthPosition();
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
        //past counts
        double proxPrev = 0;
        double distPrev = 0;
        //
        double proxAmps = 0;
        double distAmps = 0;
        //
        double[] operatorInput = {0,0};
        boolean stowed = true;

        ArmModes armmode = ArmModes.SAFETY_CATCH;
    }

    public enum ArmStates {
        // Prox, Dist bonehead
        FWD_GROUND_CARGO(-1400, 1181),
        FWD_LOW_HATCH(-1472, 650),
        FWD_LOW_CARGO(-1332, 550),
        FWD_MEDIUM_HATCH(-747, 557),
        FWD_MEDIUM_CARGO(-609, 550),
        FWD_HIGH_HATCH(-65, 558),
        FWD_HIGH_CARGO(-199, 307),

        REV_MEDIUM_HATCH(-460, -1036),
        REV_MEDIUM_CARGO(-554, -882),
        REV_HIGH_HATCH(-237, -567),
        REV_HIGH_CARGO(-236, -415),
        REV_GROUND_CARGO(1000, -200),

        CARGO_SHIP_CARGO(-535, 1000),

        GROUND_HATCH(1248, -2477),
        CLIMB_TRANSPORT(-1397, -161),
        CLIMB_READY(1118, -2400),
        UNSTOW_ARM(-626, 1400),
        STOW_ARM(-846, 2106);


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


