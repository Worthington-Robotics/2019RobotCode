package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.trajectory.TrajectoryIterator;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.util.HIDHelper;
import frc.lib.util.DriveSignal;
import frc.robot.Constants;

public class Drive extends Subsystem {

    private static Drive m_DriveInstance = new Drive();
    private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;
    private DriveMotionPlanner mMotionPlanner;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter;
    private PeriodicIO periodic;
    private Rotation2d mGyroOffset;
    private DoubleSolenoid trans;
    private boolean mOverrideTrajectory = false;
    private WPI_TalonSRX driveFrontLeft;
    private WPI_TalonSRX driveMiddleLeft;
    private WPI_TalonSRX driveBackLeft;
    private WPI_TalonSRX driveFrontRight;
    private WPI_TalonSRX driveMiddleRight;
    private WPI_TalonSRX driveBackRight;
    private double[] operatorInput = {0,0,0};
    private AHRS ahrs;
    private int ramp_Up_Counter = 0;
    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this); {
                startLogging();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                if (Constants.ENABLE_MP_TEST_MODE && DriverStation.getInstance().isTest())
                    mDriveControlState = DriveControlState.PROFILING_TEST;
                switch (mDriveControlState) {
                    case PATH_FOLLOWING:
                        updatePathFollower();
                        break;
                    case PROFILING_TEST:
                        if (Constants.RAMPUP) {
                            periodic.left_demand = -ramp_Up_Counter * 0.0025;
                            periodic.right_demand = -ramp_Up_Counter * 0.0025;
                            ramp_Up_Counter++;
                        } else if (DriverStation.getInstance().isTest()) {
                            periodic.left_demand = -Constants.MP_TEST_SPEED * Constants.TICKS_TO_INCHES;
                            periodic.right_demand = -Constants.MP_TEST_SPEED * Constants.TICKS_TO_INCHES;
                        }
                        break;
                    case OPEN_LOOP:
                        if (DriverStation.getInstance().isOperatorControl())
                            operatorInput = HIDHelper.getAdjStick(Constants.MASTER_STICK);
                        else operatorInput = new double []{0,0,0};
                        SmartDashboard.putNumberArray("stick", operatorInput);
                        setOpenLoop(arcadeDrive(operatorInput[1], operatorInput[2]));
                        break;
                    default:
                        System.out.println("Unexpected Control State");
                }
            }
        }

        @Override
        public void onStop(double timestamp) {

        }
    };
    private Drive() {
        periodic = new periodicIO();
        mMotionPlanner = new DriveMotionPlanner();
        mCSVWriter = new ReflectingCSVWriter<periodicIO>("", PeriodicIO.class);
        driveFrontLeft = new WPI_TalonSRX(Constants.DRIVE_FRONT_LEFT_ID);
        driveMiddleLeft = new WPI_TalonSRX(Constants.DRIVE_MIDDLE_LEFT_ID);
        driveBackLeft = new WPI_TalonSRX(Constants.DRIVE_BACK_LEFT_ID);
        driveFrontRight = new WPI_TalonSRX(Constants.DRIVE_FRONT_RIGHT_ID);
        driveMiddleRight = new WPI_TalonSRX(Constants.DRIVE_MIDDLE_RIGHT_ID);
        driveBackRight = new WPI_TalonSRX(Constants.DRIVE_BACK_RIGHT_ID);
        Ahrs = new AHRS(SPI.Port.kMXP);
        trans = new DoubleSolenoid(Constants.TRANS_LOW_ID,Constants.TRANS_HIGH_ID);
    }
    public static Drive getInstance() {return m_DriveInstance}
    private static double rotationsToInches(double rotations) {return rotations / Constants.ROTATIONS_TO_INCHES;}
    private static double rpmToIncehsPerSecond(double rpm) {return rotationsToInches(rpm) / 60;}
    private static double inchesToRotations(double inches) {return inches * Constants.ROTATIONS_TO_INCHES;}

}
