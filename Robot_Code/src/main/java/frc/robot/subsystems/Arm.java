package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Rotation2d;
import frc.lib.loops.Loop;
import frc.lib.util.DriveSignal;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;
import frc.robot.planners.ArmMotionPlanner;

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
    private ArmMotionPlanner armMotionPlanner;
    private boolean mOverrideTrajectory = false;

    private Arm() {
        armProx = new TalonSRX(Constants.ARM_PRONOMINAL);
        armDist = new TalonSRX(Constants.ARM_DISTAL);
        armProx.configContinuousCurrentLimit(20);
        armDist.configContinuousCurrentLimit(20);
        armDist.enableCurrentLimit(true);
        armProx.enableCurrentLimit(true);
        //US1 = new Ultrasonic(Constants.ULTRASONIC_IN_1, Constants.ULTRASONIC_OUT_1);
        //US2 = new Ultrasonic(Constants.ULTRASONIC_IN_2, Constants.ULTRASONIC_OUT_2);
        armMotionPlanner = new ArmMotionPlanner();
        reset();
    }

    private Loop m_loop = new Loop() {
        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {
            switch (periodic.armmode) {
                case PID: break;
                case VELOCITY_CONTROL: break;
            }
        }

        @Override
        public void onStop(double timestamp) {

        }
    };

    public void readPeriodicInputs() {
        periodic.operatorInput = HIDHelper.getAdjStick(Constants.LAUNCHPAD_STICK);
        periodic.sideShift = Constants.LAUNCH_PAD.getRawButton(9);
        periodic.proxPrev = periodic.proxRel;
        periodic.distPrev = periodic.distRel;
        //periodic.US1Past = periodic.US1Dis;
        //periodic.US2Past = periodic.US2Dis;
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
        if (periodic.armmode == ArmModes.DIRECT_CONTROL) {
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
            case DIRECT_CONTROL:
                armProx.set(ControlMode.PercentOutput, periodic.operatorInput[0]);
                armDist.set(ControlMode.PercentOutput, periodic.operatorInput[1]);
                break;
            case PID:
                armProx.set(ControlMode.Position, periodic.armProxPower + periodic.proxMod, DemandType.ArbitraryFeedForward, Constants.ARM_PROX_A_FEEDFORWARD * Math.sin((periodic.armProxPower + periodic.proxMod) / 2048 * Math.PI));
                armDist.set(ControlMode.Position, periodic.armDistPower + periodic.distMod/*, DemandType.ArbitraryFeedForward, Math.sin(periodic.armDistPower + periodic.distMod / 2048 * Math.PI)*/);
                break;
            case SAFETY_CATCH:
                armProx.set(ControlMode.PercentOutput, 0);
                armDist.set(ControlMode.PercentOutput, 0);
                break;
            case VELOCITY_CONTROL:
                armProx.set(ControlMode.Velocity, periodic.armProxPower, DemandType.ArbitraryFeedForward, (periodic.proxVoltage + Constants.ARM_PROX_KD * periodic.proxAccel / 1023.0));
                armDist.set(ControlMode.Velocity, periodic.armDistPower, DemandType.ArbitraryFeedForward, (periodic.distVoltage + Constants.ARM_DIST_KD * periodic.distAccel / 1023.0));
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
        //
        SmartDashboard.putNumber("Arm/Dist Mod", periodic.distMod);
        SmartDashboard.putNumber("Arm/Distal Arm Power", periodic.armDistPower);
        SmartDashboard.putNumber("Arm/Distal Arm Error", periodic.distError);
        SmartDashboard.putNumber("Arm/Dist Rel", periodic.distRel);
        SmartDashboard.putNumber("Arm/Dist Point", periodic.distRel - periodic.distMod);
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
        armProx.setSelectedSensorPosition(0);
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
        armDist.setSelectedSensorPosition(0);
    }

    public void setPIDArmConfig(ArmStates state) {
        if (periodic.armmode != ArmModes.PID) {
            periodic.armmode = ArmModes.PID;
        }
        periodic.armProxPower = state.prox;
        periodic.armDistPower = state.dist;
    }


    public void safeMode() {
        periodic.armmode = ArmModes.SAFETY_CATCH;
    }

    public void setVelocitymConfig(double prox, double dist) {
        if (periodic.armmode != ArmModes.DIRECT_CONTROL) {
            periodic.armmode = ArmModes.DIRECT_CONTROL;
        }
        periodic.armProxPower = prox;
        periodic.armDistPower = dist;
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

    public Rotation2d ticksToRotation2D(double ticks) {
        double radians = (ticks/4096) * (2 * Math.PI);
        return Rotation2d.fromRadians(radians);
    }

    public void updateArmMotionPlanner() {
        final double now = Timer.getFPGATimestamp();
        ArmMotionPlanner.Output armMotionPlannerOutput = armMotionPlanner.update(now,
                ticksToRotation2D(periodic.proxRel - periodic.proxMod),
                ticksToRotation2D(periodic.distRel - periodic.distMod));
        periodic.armProxPower = armMotionPlannerOutput.proxVel;
        periodic.armDistPower = armMotionPlannerOutput.distVel;
        periodic.proxVoltage = armMotionPlannerOutput.proxVoltage;
        periodic.distVoltage = armMotionPlannerOutput.distVoltage;
        periodic.proxAccel = armMotionPlannerOutput.proxAccel;
        periodic.distAccel = armMotionPlannerOutput.distAccel;
    }
    public synchronized void setTrajectory(ArmMotionPlanner.ArmTrajectory trajectory) {
        if (armMotionPlanner != null) {
            mOverrideTrajectory = false;
            armMotionPlanner.reset();
            armMotionPlanner.setTrajectory(trajectory);
            periodic.armmode = ArmModes.VELOCITY_CONTROL;
        }
    }

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
        DIRECT_CONTROL,
        PID,
        VELOCITY_CONTROL,
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
        //double US1Past = 0;
        //double US2Past = 0;
        //double US1Dis = 0;
        //double US2Dis = 0;
        //TALON RELS
        double proxRel = 0;
        double distRel = 0;
        //past counts
        double proxPrev = 0;
        double distPrev = 0;
        //
        double proxAmps = 0;
        double distAmps = 0;
        //V Control Things
        double proxAccel = 0;
        double distAccel = 0;
        double proxVoltage = 0;
        double distVoltage = 0;
        //
        double[] operatorInput = {0,0};
        boolean stowed = true;

        ArmModes armmode = ArmModes.SAFETY_CATCH;
    }

    public enum ArmStates {
        // Prox, Dist bonehead
        FWD_GROUND_CARGO(-1324, 1308),
        FWD_LOW_HATCH(-1467, 795),
        FWD_LOW_CARGO(-1464, 649),
        FWD_MEDIUM_HATCH(-861, 670),
        FWD_MEDIUM_CARGO(-691, 631),
        FWD_HIGH_HATCH(-268, 450),
        FWD_HIGH_CARGO(-339, 284),

        REV_MEDIUM_HATCH(-460, -1036),
        REV_MEDIUM_CARGO(-554, -882),
        REV_HIGH_HATCH(-237, -567),
        REV_HIGH_CARGO(-236, -415),
        REV_GROUND_CARGO(1000, -200),

        GROUND_HATCH(1248, -2477),
        CLIMB_TRANSPORT(-100,-1364),
        UNSTOW_ARM(-600, 1500),
        STOW_ARM(-1000, -700);

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


