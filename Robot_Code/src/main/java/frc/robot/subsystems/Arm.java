package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;

/**
 * ~~The code for governing the three-axis arm~~
 * ~~The code for governing the two-axis arm~~
 * ~~The code for governing the one-and-a-half axis arm~~
 * The code for governing our arm
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

    private TalonSRX /*armProx,*/ armDist;
    private DoubleSolenoid proxPist;
    private PeriodicIO periodic;
    //private Ultrasonic US1, US2;
    private double[] operatorInput;

    private Arm() {
        armDist = new TalonSRX(Constants.ARM_DISTAL);
        proxPist = new DoubleSolenoid(Constants.PROX_LOW, Constants.PROX_HIGH);
        reset();
    }

    public void readPeriodicInputs() {

        periodic.operatorInput = HIDHelper.getAdjStick(Constants.LAUNCHPAD_STICK);
        periodic.sideShift = Constants.LAUNCH_PAD.getRawButton(9);
        periodic.distAmps = armDist.getOutputCurrent();
        periodic.distError = armDist.getClosedLoopError();
        periodic.distRel = armDist.getSensorCollection().getQuadraturePosition();
        periodic.distAbsolute = armDist.getSensorCollection().getPulseWidthPosition();
        periodic.distPoint = periodic.distRel - periodic.distMod;
    }


    public void writePeriodicOutputs() {

        if (periodic.ProxPiston.equals(DoubleSolenoid.Value.kReverse)) {
            switch (periodic.armmode) {
                case DirectControl:
                    //armProx.set(ControlMode.PercentOutput, periodic.operatorInput[0]);
                    if (periodic.operatorInput[1] > 0 && periodic.distPoint > Constants.ARM_U_U_LIMIT) {
                        proxPist.set(periodic.ProxPiston);
                        armDist.set(ControlMode.PercentOutput, 0);
                        break;
                    }
                    if (periodic.operatorInput[1] < 0 && periodic.distPoint < Constants.ARM_U_L_LIMIT) {
                        proxPist.set(periodic.ProxPiston);
                        armDist.set(ControlMode.PercentOutput, 0);
                    } else {
                        armDist.set(ControlMode.PercentOutput, periodic.operatorInput[1]);
                        proxPist.set(periodic.ProxPiston);
                        break;
                    }
            /*case PID:
                armProx.set(ControlMode.Position, periodic.armProxPower + periodic.proxMod/* + (periodic.operatorInput[0]*100), DemandType.ArbitraryFeedForward, Constants.ARM_PROX_A_FEEDFORWARD * Math.sin((periodic.armProxPower + periodic.proxMod) / 2048 * Math.PI));
                armDist.set(ControlMode.Position, periodic.armDistPower + periodic.distMod /*+ (periodic.operatorInput[1]*100) /*DemandType.ArbitraryFeedForward, Constants.ARM_DIST_A_FEEDFORWARD * Math.sin((periodic.armDistPower + periodic.distMod + periodic.proxMod + periodic.armProxPower) / 2048 * Math.PI));
                break;*/

                case PPID:
                    //armProx.set(ControlMode.Velocity, 0);
                    if (periodic.distPoint > Constants.ARM_U_U_LIMIT) {
                        periodic.armmode = ArmModes.DirectControl;
                        break;
                    }
                    if (periodic.distPoint < Constants.ARM_U_L_LIMIT) {
                        periodic.armmode = ArmModes.DirectControl;
                    } else {
                        armDist.set(ControlMode.Position, periodic.armDistPower + periodic.distMod /*+ (periodic.operatorInput[1]*100) /*DemandType.ArbitraryFeedForward, Constants.ARM_DIST_A_FEEDFORWARD * Math.sin((periodic.armDistPower + periodic.distMod + periodic.proxMod + periodic.armProxPower) / 2048 * Math.PI)*/);
                        break;
                    }
                case STATE_SPACE:

                    break;
                case SAFETY_CATCH:
                    //armProx.set(ControlMode.PercentOutput, 0);
                    armDist.set(ControlMode.PercentOutput, 0);
                    break;
                default:
                    System.out.println("Arm entered unexpected control state!");
            }
        } else {
            switch (periodic.armmode) {
                case DirectControl:
                            //armProx.set(ControlMode.PercentOutput, periodic.operatorInput[0]);
                    if (periodic.operatorInput[1] > 0 && periodic.distPoint > Constants.ARM_L_U_LIMIT) {
                        proxPist.set(periodic.ProxPiston);
                        armDist.set(ControlMode.PercentOutput, 0);
                        break;
                    }
                    if (periodic.operatorInput[1] < 0 && periodic.distPoint < Constants.ARM_L_L_LIMIT) {
                        proxPist.set(periodic.ProxPiston);
                        armDist.set(ControlMode.PercentOutput, 0);
                    } else {
                        armDist.set(ControlMode.PercentOutput, periodic.operatorInput[1]);
                        proxPist.set(periodic.ProxPiston);
                        break;
                    }
            /*case PID:
                armProx.set(ControlMode.Position, periodic.armProxPower + periodic.proxMod/* + (periodic.operatorInput[0]*100), DemandType.ArbitraryFeedForward, Constants.ARM_PROX_A_FEEDFORWARD * Math.sin((periodic.armProxPower + periodic.proxMod) / 2048 * Math.PI));
                armDist.set(ControlMode.Position, periodic.armDistPower + periodic.distMod /*+ (periodic.operatorInput[1]*100) /*DemandType.ArbitraryFeedForward, Constants.ARM_DIST_A_FEEDFORWARD * Math.sin((periodic.armDistPower + periodic.distMod + periodic.proxMod + periodic.armProxPower) / 2048 * Math.PI));
                break;*/
                    case PPID:
                    //armProx.set(ControlMode.Velocity, 0);
                        if (periodic.distPoint > Constants.ARM_L_U_LIMIT) {
                            periodic.armmode = ArmModes.DirectControl;
                            break;
                        }
                        if (periodic.distPoint < Constants.ARM_L_L_LIMIT) {
                            periodic.armmode = ArmModes.DirectControl;
                        } else {
                            armDist.set(ControlMode.Position, periodic.armDistPower + periodic.distMod /*+ (periodic.operatorInput[1]*100) /*DemandType.ArbitraryFeedForward, Constants.ARM_DIST_A_FEEDFORWARD * Math.sin((periodic.armDistPower + periodic.distMod + periodic.proxMod + periodic.armProxPower) / 2048 * Math.PI)*/);
                            break;
                        }
                case STATE_SPACE:

                            break;
                        case SAFETY_CATCH:
                            //armProx.set(ControlMode.PercentOutput, 0);
                            armDist.set(ControlMode.PercentOutput, 0);
                            break;
                        default:
                            System.out.println("Arm entered unexpected control state!");
                    }
        }
    }

    public void outputTelemetry() {
        /*SmartDashboard.putNumber("Arm/Prox Mod", periodic.proxMod);
        SmartDashboard.putNumber("Arm/Proximal Arm Power", periodic.armProxPower);
        //martDashboard.putNumber("Arm/Proximal Arm Error", periodic.proxError);
        SmartDashboard.putNumber("Arm/Prox Rel", periodic.proxRel);
        SmartDashboard.putNumber("Arm/Prox Point", periodic.proxRel - periodic.proxMod);
        SmartDashboard.putNumber("Arm/Prox Dial", periodic.operatorInput[0]);
        //SmartDashboard.putNumber("Arm/Prox Abs", periodic.proxAbsolute);*/
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
        configTalons();
        resetArmMod();
    }

    public void resetArmMod() {
        //periodic.proxMod = Constants.PROX_ABSOLUTE_ZERO - periodic.proxAbsolute;
        periodic.distMod = Constants.DIST_ABSOLUTE_ZERO - periodic.distAbsolute;
    }

    public void configTalons() {
        boolean proxCal = true;
        /*armProx.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        armProx.setSensorPhase(proxCal);
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
        armProx.setSensorPhase(proxCal);
        armProx.setSelectedSensorPosition(0);*/
        //
        armDist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        armDist.setSensorPhase(false);
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

    /*public void setPIDArmConfig(ArmStates state) {
        if (periodic.armmode != ArmModes.PID) {
            periodic.armmode = ArmModes.PID;
        }
        periodic.armstate = state;
        periodic.armProxPower = state.prox;
        periodic.armDistPower = state.dist;
    }*/

    public void setPistPIDArmConfig(PistonArmStates state) {
        if (periodic.armmode != ArmModes.PPID) {
            periodic.armmode = ArmModes.PPID;
        }
        periodic.PArmStates = state;
        if (state.getProx()) {
            setProxPistPID(DoubleSolenoid.Value.kReverse);
        }
            else
        {
            setProxPistPID(DoubleSolenoid.Value.kForward);
        }
        periodic.armDistPower = state.dist;
    }

    /*public ArmStates getArmState() {
        if (periodic.armmode.equals(ArmModes.PID)) return periodic.armstate;
        return null;
    }*/

   /* public void setSSArmConfig(ArmStates state) {
        if (periodic.armmode != ArmModes.STATE_SPACE) {
            periodic.armmode = ArmModes.STATE_SPACE;
        }
        periodic.armProxPower = state.prox;
        periodic.armDistPower = state.dist;
    }
*/
    public void safeMode() {
        if (periodic.armmode != ArmModes.SAFETY_CATCH)
            periodic.armmode = ArmModes.SAFETY_CATCH;
    }

    public void setVelocitymConfig() {
        if (periodic.armmode != ArmModes.DirectControl) {
            periodic.armmode = ArmModes.DirectControl;
        }
        periodic.ProxPiston = DoubleSolenoid.Value.kOff;
    }

    /*public double getProxPoint() {
        return periodic.proxRel - periodic.proxMod;
    }*/

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
    public boolean getStowed() {
        return periodic.stowed;
    }

    public void setStowed(boolean stowed) {
        periodic.stowed = stowed;
    }

    public boolean getSideShift() {
        return periodic.sideShift;
    }

    public double getAbsolute() {
        return periodic.distAbsolute;
    }
    public boolean getProxExtend(){
        return periodic.ProxPiston.equals(DoubleSolenoid.Value.kForward);
    }

    public void setProxPist(DoubleSolenoid.Value proxPist) {
        periodic.ProxPiston = proxPist;
    }

    public void setProxPistPID(DoubleSolenoid.Value proxPist) {
        this.proxPist.set(proxPist);
    }


    public enum ArmModes {
        DirectControl,
        /*PID,*/
        PPID,
        STATE_SPACE,
        SAFETY_CATCH;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    public class PeriodicIO {
        //Side Shift
        boolean sideShift = false;
        double armDistPower = 0;
        DoubleSolenoid.Value ProxPiston = DoubleSolenoid.Value.kReverse;
        double distAbsolute = armDist.getSensorCollection().getPulseWidthPosition();
        double distMod = 0;
        double distError = armDist.getClosedLoopError();
        double distRel = 0;
        double distAmps = 0;
        double[] operatorInput = {0, 0};
        boolean stowed = true;
        //ArmStates armstate = ArmStates.STOW_ARM;
        PistonArmStates PArmStates = PistonArmStates.STOW_ARM;
        ArmModes armmode = ArmModes.SAFETY_CATCH;
        double distPoint =  distRel - distMod;
    }

   /* public enum ArmStates {
        // Prox, Dist bonehead
        // +60
        DIST_PICKUP(0, -335),
        DIST_CARGOSHIP(0, 640),
        FWD_GROUND_CARGO(-1287, -242),
        FWD_LOW_CARGO(-1232, 356),// 468 add 100 dis for gravity
        FWD_MEDIUM_CARGO(-323, -788),
        FWD_HIGH_CARGO(-410, -46),
        CARGO_SHIP_CARGO(-629, -589),
        A_CARGO_SHIP_CARGO(-393, -1111),
        UNSTOW_ARM(-568, -971), //1556
        CLIMB_MID_CHECK(-1024, 0),
        STOW_ARM(-1043, -1059);


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
    }*/

    public enum PistonArmStates {
        // Prox, Dist bonehead
        // +60
        // true is piston extended
        FWD_GROUND_CARGO(false, -200),
        FWD_LOW_CARGO(false, 367),// 468 add 100 dis for gravity
        FWD_MEDIUM_CARGO(true, 265),
        CARGO_SHIP_CARGO(true, 0),
        UNSTOW_ARM(true, 0),
        STOW_ARM(true, -1059);


        private double dist;
        private boolean prox;

        PistonArmStates(boolean prox, double dist) {
            this.prox = prox;
            this.dist = dist;
        }

        public boolean getProx() {
            return prox;
        }

        public double getDist() {
            return dist;
        }
    }
}


