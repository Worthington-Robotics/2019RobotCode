package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

    public TalonSRX armDist;
    private DoubleSolenoid proxPist;
    private PeriodicIO periodic;

    private Arm() {
        armDist = new TalonSRX(Constants.ARM_DISTAL);
        proxPist = new DoubleSolenoid(Constants.PROX_LOW, Constants.PROX_HIGH);
        reset();
    }

    public void readPeriodicInputs() {

        periodic.StowSwitch = armDist.getSensorCollection().isRevLimitSwitchClosed();
        periodic.operatorInput = HIDHelper.getAdjStick(Constants.LAUNCHPAD_STICK);
        periodic.sideShift = Constants.LAUNCH_PAD.getRawButton(9);
        periodic.distAmps = armDist.getOutputCurrent();
        periodic.distError = armDist.getClosedLoopError();
        periodic.distRel = armDist.getSensorCollection().getQuadraturePosition();
        periodic.distAbsolute = armDist.getSensorCollection().getPulseWidthPosition();
        periodic.distPoint = periodic.distRel - periodic.distMod;
        if(Constants.LAUNCH_PAD.getRawButton(9) && !periodic.SToggle)
        {
            periodic.IgnoreSafety = !periodic.IgnoreSafety;
            periodic.SToggle = true;
        }
        else if(!Constants.LAUNCH_PAD.getRawButton(9))
        {
            periodic.SToggle = false;
        }
        if(periodic.StowSwitch)
        {
            System.out.println("Stowed Switch");
            Constants.DIST_ABSOLUTE_ZERO = periodic.distAbsolute - Arm.PistonArmStates.STOW_ARM.getDist();
            System.out.println("Zero Calced");
            resetArmMod();
            System.out.println("Mod Adjusted");
        }
    }


    public void writePeriodicOutputs() {
        if (!periodic.ProxPiston.equals(DoubleSolenoid.Value.kReverse)) {
            switch (periodic.armmode) {
                case DirectControl:
                    if (periodic.operatorInput[1] > 0 && periodic.distPoint > Constants.ARM_U_U_LIMIT && !periodic.IgnoreSafety) {
                        proxPist.set(periodic.ProxPiston);
                        armDist.set(ControlMode.PercentOutput, 0);
                        break;
                    } else {
                        proxPist.set(periodic.ProxPiston);
                        armDist.set(ControlMode.PercentOutput, periodic.operatorInput[1]);
                        break;
                    }

                case PPID:
                    if (periodic.distPoint > Constants.ARM_U_U_LIMIT && !periodic.IgnoreSafety) {
                        periodic.armmode = ArmModes.DirectControl;
                        break;
                    }
                    else if (periodic.distPoint < Constants.ARM_U_L_LIMIT && !periodic.IgnoreSafety) {
                        periodic.armmode = ArmModes.DirectControl;
                    } else {
                        armDist.set(ControlMode.Position, periodic.armDistPower + periodic.distMod /*+ (periodic.operatorInput[1]*100) /*DemandType.ArbitraryFeedForward, Constants.ARM_DIST_A_FEEDFORWARD * Math.sin((periodic.armDistPower + periodic.distMod + periodic.proxMod + periodic.armProxPower) / 2048 * Math.PI)*/);
                        break;
                    }
                case STATE_SPACE:

                    break;
                case SAFETY_CATCH:
                    armDist.set(ControlMode.PercentOutput, 0);
                    break;
                default:
                    System.out.println("Arm entered unexpected control state!");
            }
        } else {
            switch (periodic.armmode) {
                case DirectControl:
                    if (periodic.operatorInput[1] > 0 && periodic.distPoint > Constants.ARM_L_U_LIMIT && !periodic.IgnoreSafety) {
                        proxPist.set(periodic.ProxPiston);
                        armDist.set(ControlMode.PercentOutput, 0);
                        break;
                    }
                    else if (periodic.operatorInput[1] < 0 && periodic.distPoint < Constants.ARM_L_L_LIMIT && !periodic.IgnoreSafety) {
                        proxPist.set(periodic.ProxPiston);
                        armDist.set(ControlMode.PercentOutput, 0);
                    } else {
                        proxPist.set(periodic.ProxPiston);
                        armDist.set(ControlMode.PercentOutput, periodic.operatorInput[1]);
                        break;
                    }
                case PPID:
                    if (periodic.distPoint > Constants.ARM_L_U_LIMIT && !periodic.IgnoreSafety) {
                        periodic.armmode = ArmModes.DirectControl;
                        break;
                    }
                    else if (periodic.distPoint < Constants.ARM_L_L_LIMIT && !periodic.IgnoreSafety) {
                        periodic.armmode = ArmModes.DirectControl;
                    } else {
                        armDist.set(ControlMode.Position, periodic.armDistPower + periodic.distMod /*+ (periodic.operatorInput[1]*100) /*DemandType.ArbitraryFeedForward, Constants.ARM_DIST_A_FEEDFORWARD * Math.sin((periodic.armDistPower + periodic.distMod + periodic.proxMod + periodic.armProxPower) / 2048 * Math.PI)*/);
                        break;
                    }
                case STATE_SPACE:

                    break;
                case SAFETY_CATCH:
                    armDist.set(ControlMode.PercentOutput, 0);
                    break;
                default:
                    System.out.println("Arm entered unexpected control state!");
            }
        }
    }

    public void outputTelemetry() {
        SmartDashboard.putBoolean("Arm/Ignore Safety", periodic.IgnoreSafety);
        SmartDashboard.putNumber("Arm/Dist Mod", periodic.distMod);
        SmartDashboard.putNumber("Arm/Distal Arm Power", periodic.armDistPower);
        SmartDashboard.putNumber("Arm/Distal Arm Error", periodic.distError);
        SmartDashboard.putNumber("Arm/Dist Rel", periodic.distRel);
        SmartDashboard.putNumber("Arm/Dist Point", periodic.distRel - periodic.distMod);
        SmartDashboard.putNumber("Arm/Dist Dial", periodic.operatorInput[1]);
        SmartDashboard.putNumber("Arm/Dist Abs", periodic.distAbsolute);
        SmartDashboard.putString("Arm/Mode", periodic.armmode.toString());
        SmartDashboard.putBoolean("Arm/Stowed", periodic.stowed);
        SmartDashboard.putBoolean("Arm/Stowed Switch", periodic.StowSwitch);
    }


    public void reset() {
        periodic = new PeriodicIO();
        configTalons();
        resetArmMod();
    }

    public void resetArmMod() {
        periodic.distMod = Constants.DIST_ABSOLUTE_ZERO - periodic.distAbsolute;
    }

    public void configTalons() {
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

    public void setPistPIDArmConfig(PistonArmStates state) {
        if (periodic.armmode != ArmModes.PPID) {
            periodic.armmode = ArmModes.PPID;
        }
        periodic.PArmStates = state;
        if (state.getProx()) {
            setProxPistPID(DoubleSolenoid.Value.kForward);
        } else if ((periodic.ProxPiston.equals(DoubleSolenoid.Value.kForward) && periodic.distPoint <= Constants.ARM_NO_DOWN_LIMIT)) {
        } else {
            setProxPistPID(DoubleSolenoid.Value.kReverse);
        }
        periodic.armDistPower = state.dist;
    }


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

    public double getDistPoint() {
        return periodic.distRel - periodic.distMod;
    }


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

    public boolean getProxExtend() {
        return periodic.ProxPiston.equals(DoubleSolenoid.Value.kForward);
    }

    public void setProxPist(DoubleSolenoid.Value proxPist) {
        if ((periodic.ProxPiston.equals(DoubleSolenoid.Value.kReverse) && periodic.distPoint <= Constants.ARM_NO_DOWN_LIMIT)) {
        } else {
            periodic.ProxPiston = proxPist;
        }
    }

    public void setProxPistPID(DoubleSolenoid.Value proxPist) {
        this.proxPist.set(proxPist);
    }

    public void setIgnoreSafety(boolean n)
    {
        periodic.IgnoreSafety = n;
    }
    public void eh()
    {
        periodic.armmode = ArmModes.Eh;
    }


    public enum ArmModes {
        DirectControl,
        /*PID,*/
        PPID,
        STATE_SPACE,
        Eh,
        SAFETY_CATCH;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    public class PeriodicIO {
        //Side Shift
        boolean sideShift = false;
        double armDistPower = 0;
        DoubleSolenoid.Value ProxPiston = DoubleSolenoid.Value.kForward;
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
        double distPoint = distRel - distMod;
        boolean IgnoreSafety = false;
        boolean SToggle = false;
        boolean StowSwitch = false;
    }



    public enum PistonArmStates {
        // Prox, Dist bonehead
        // +60
        // true is piston extended
        FWD_GROUND_CARGO(false, -350),
        FWD_LOW_CARGO(false, 367),// 468 add 100 dis for gravity
        FWD_MEDIUM_CARGO(true, 265),
        FWD_HIGH_CARGO(true, 650),
        CARGO_SHIP_CARGO(false, 580),
        A_CARGO_SHIP_CARGO(true, -207),
        UNSTOW_ARM(true, 0),
        STOW_ARM(true, -1541);


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


