package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU;
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
import frc.lib.util.DriveSignal;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;
import frc.robot.planners.DriveMotionPlanner;

public class Drive extends Subsystem {

    //used internally for data

    //construct one and only 1 instance of this class
    private static Drive m_DriveInstance = new Drive();
    private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;
    private DriveMotionPlanner mMotionPlanner;
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
    private double[] operatorInput = {0, 0, 0}; //last input set from joystick update
    private PigeonIMU pigeonIMU;
    private int ramp_Up_Counter = 0;
    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
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
                            periodic.left_demand = -ramp_Up_Counter * .0025;
                            periodic.right_demand = -ramp_Up_Counter * .0025;
                            ramp_Up_Counter++;
                        } else if (DriverStation.getInstance().isTest()) {
                            periodic.left_demand = -Constants.MP_TEST_SPEED * Constants.TICKS_TO_INCHES;
                            periodic.right_demand = -Constants.MP_TEST_SPEED * Constants.TICKS_TO_INCHES;
                        }

                        break;


                    case OPEN_LOOP:
                        if (DriverStation.getInstance().isOperatorControl())
                            operatorInput = HIDHelper.getAdjStick(Constants.MASTER_STICK);

                        else operatorInput = new double[]{0, 0, 0};
                        SmartDashboard.putNumberArray("stick", operatorInput);
                        setOpenLoop(arcadeDrive(operatorInput[1], operatorInput[2]));
                        break;

                    default:
                        System.out.println("You fool, unexpected control state");
                }


            }
        }

        @Override
        public void onStop(double timestamp) {

        }
    };

    private Drive() {
        periodic = new PeriodicIO();
        mMotionPlanner = new DriveMotionPlanner();
        driveFrontLeft = new WPI_TalonSRX(Constants.DRIVE_FRONT_LEFT_ID);
        driveMiddleLeft = new WPI_TalonSRX(Constants.DRIVE_MIDDLE_LEFT_ID);
        driveBackLeft = new WPI_TalonSRX(Constants.DRIVE_BACK_LEFT_ID);
        driveFrontRight = new WPI_TalonSRX(Constants.DRIVE_FRONT_RIGHT_ID);
        driveMiddleRight = new WPI_TalonSRX(Constants.DRIVE_MIDDLE_RIGHT_ID);
        driveBackRight = new WPI_TalonSRX(Constants.DRIVE_BACK_RIGHT_ID);
        pigeonIMU = new PigeonIMU(driveFrontLeft);
        trans = new DoubleSolenoid(Constants.TRANS_LOW_ID, Constants.TRANS_HIGH_ID);

    }


    public static Drive getInstance() {
        return m_DriveInstance;
    }

    private static double rotationsToInches(double rotations) {
        return rotations / Constants.ROTATIONS_TO_INCHES;
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches * Constants.ROTATIONS_TO_INCHES;
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
    }

    private static double rpmToTicksPer100ms(double rpm) {
        return ((rpm * 512.0) / 75.0);
    }

    public synchronized Rotation2d getHeading() {
        return periodic.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("SET HEADING: " + heading.getDegrees());

        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(0).inverse()); //TODO replace zero with gyro source
        System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

        periodic.gyro_heading = heading;
    }

    public double getLeftEncoderRotations() {
        return periodic.left_pos_ticks / Constants.DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return periodic.right_pos_ticks / Constants.DRIVE_ENCODER_PPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    public double getLeftVelocityNativeUnits() {
        return periodic.left_velocity_ticks_per_100ms;
    }

    public double getRightVelocityNativeUnits() {
        return periodic.right_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / Constants.DRIVE_ENCODER_PPR);
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / Constants.DRIVE_ENCODER_PPR);
    }

    public void setOperatorInput(double[] input) {
        operatorInput = input;
    }

    public void reset() {
        mOverrideTrajectory = false;


        mMotionPlanner.reset();
        mMotionPlanner.setFollowerType(DriveMotionPlanner.FollowerType.PURE_PURSUIT);
        pigeonIMU.hasResetOccurred();
        periodic = new PeriodicIO();
        periodic.right_pos_ticks = 0;
        periodic.left_pos_ticks = 0;
        ramp_Up_Counter = 0;
        driveFrontRight.setSelectedSensorPosition(0, 0, 0);
        driveFrontLeft.setSelectedSensorPosition(0, 0, 0);
        driveFrontLeft.setSensorPhase(false);
        driveFrontRight.setSensorPhase(true);
        driveFrontLeft.selectProfileSlot(0, 0);
        driveFrontLeft.config_kF(0, Constants.DRIVE_LEFT_KF, 0);
        driveFrontLeft.config_kP(0, Constants.DRIVE_LEFT_KP, 0);
        driveFrontLeft.config_kI(0, Constants.DRIVE_LEFT_KI, 0);
        driveFrontLeft.config_kD(0, Constants.DRIVE_LEFT_KD, 0);
        driveFrontLeft.config_IntegralZone(0, 0, 0);
        driveFrontRight.selectProfileSlot(0, 0);
        driveFrontRight.config_kF(0, Constants.DRIVE_RIGHT_KF, 0);
        driveFrontRight.config_kP(0, Constants.DRIVE_RIGHT_KP, 0);
        driveFrontRight.config_kI(0, Constants.DRIVE_RIGHT_KI, 0);
        driveFrontRight.config_kD(0, Constants.DRIVE_RIGHT_KD, 0);
        driveFrontRight.config_IntegralZone(0, 0, 0);
        driveFrontRight.setNeutralMode(NeutralMode.Brake);
        driveFrontLeft.setNeutralMode(NeutralMode.Brake);
        driveMiddleRight.setNeutralMode(NeutralMode.Brake);
        driveMiddleLeft.setNeutralMode(NeutralMode.Brake);
        driveBackRight.setNeutralMode(NeutralMode.Brake);
        driveBackLeft.setNeutralMode(NeutralMode.Brake);

        //TODO add reset with sensor impl

    }

    public void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    private void updatePathFollower() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();

            DriveMotionPlanner.Output output = mMotionPlanner.update(now, PoseEstimator.getInstance().getFieldToVehicle(now));

            periodic.error = mMotionPlanner.error();
            periodic.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory) {
                DriveSignal signal = new DriveSignal(radiansPerSecondToTicksPer100ms(inchesPerSecondToRpm(-output.right_velocity)),
                        radiansPerSecondToTicksPer100ms(inchesPerSecondToRpm(-output.left_velocity)));

                setVelocity(signal, new DriveSignal(output.left_feedforward_voltage/12, output.right_feedforward_voltage/12));
                periodic.left_accl = radiansPerSecondToTicksPer100ms(output.left_accel)/1000;
                periodic.right_accl = radiansPerSecondToTicksPer100ms(output.right_accel)/1000;

            } else {
                setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                mDriveControlState = DriveControlState.OPEN_LOOP;
                mMotionPlanner.reset();

            }
        } else {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    private void configTele() {
        reset();
        driveFrontLeft.set(ControlMode.PercentOutput, 0);
        driveMiddleLeft.set(ControlMode.Follower, driveFrontLeft.getDeviceID());
        driveBackLeft.set(ControlMode.Follower, driveFrontLeft.getDeviceID());
        driveFrontRight.set(ControlMode.PercentOutput, 0);
        driveMiddleRight.set(ControlMode.Follower, driveFrontRight.getDeviceID());
        driveBackRight.set(ControlMode.Follower, driveFrontRight.getDeviceID());
    }

    private DriveSignal arcadeDrive(double xSpeed, double zRotation) {
        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }
        return new DriveSignal(rightMotorOutput, leftMotorOutput);
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            System.out.println("Switching to open loop");
            configTele();
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        periodic.left_demand = signal.getLeft();
        periodic.right_demand = signal.getRight();
    }

    /**
     * Configures talons for velocity control
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            // We entered a velocity control state.
            //TODO configure motor control for velocity

            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
        periodic.left_demand = signal.getLeft(); //TODO convert to native units
        periodic.right_demand = signal.getRight(); //TODO convert to native units
        periodic.left_feedforward  = feedforward.getLeft();
        periodic.right_feedforward  = feedforward.getRight();


    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if (mMotionPlanner != null) {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
    }

    public boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            return true;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }


    @Override
    public synchronized void readPeriodicInputs() {
        double prevLeftTicks = periodic.left_pos_ticks;
        double prevRightTicks = periodic.right_pos_ticks;
        periodic.B2 = Constants.MASTER.getRawButton(2);
        periodic.left_pos_ticks = -driveFrontLeft.getSelectedSensorPosition(0);
        periodic.right_pos_ticks = -driveFrontRight.getSelectedSensorPosition(0);
        periodic.left_velocity_ticks_per_100ms = -driveFrontLeft.getSelectedSensorVelocity(0);
        periodic.right_velocity_ticks_per_100ms = -driveFrontRight.getSelectedSensorVelocity(0);
        periodic.gyro_heading = Rotation2d.fromDegrees(pigeonIMU.getCompassHeading());//.rotateBy(mGyroOffset);


        double deltaLeftTicks = ((periodic.left_pos_ticks - prevLeftTicks) / 4096.0) * Math.PI;
        if (deltaLeftTicks > 0.0) {
            periodic.left_distance += deltaLeftTicks * Constants.DRIVE_WHEEL_DIAMETER_INCHES;
        } else {
            periodic.left_distance += deltaLeftTicks * Constants.DRIVE_WHEEL_DIAMETER_INCHES;
        }


        double deltaRightTicks = ((periodic.right_pos_ticks - prevRightTicks) / 4096.0) * Math.PI;
        if (deltaRightTicks > 0.0) {
            periodic.right_distance += deltaRightTicks * Constants.DRIVE_WHEEL_DIAMETER_INCHES;
        } else {
            periodic.right_distance += deltaRightTicks * Constants.DRIVE_WHEEL_DIAMETER_INCHES;
        }
        // System.out.println("control state: " + mDriveControlState + ", left: " + periodic.linear_demand + ", right: " + periodic.angular_demand);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        gearShift();
        if (mDriveControlState == DriveControlState.OPEN_LOOP || (mDriveControlState == DriveControlState.PROFILING_TEST && Constants.RAMPUP)) {
            //TODO write open loop outputs
            driveFrontLeft.set(ControlMode.PercentOutput, periodic.left_demand);
            driveMiddleLeft.set(ControlMode.Follower, driveFrontLeft.getDeviceID());
            driveBackLeft.set(ControlMode.Follower, driveFrontLeft.getDeviceID());
            driveFrontRight.set(ControlMode.PercentOutput, -periodic.right_demand);
            driveMiddleRight.set(ControlMode.Follower, driveFrontRight.getDeviceID());
            driveBackRight.set(ControlMode.Follower, driveFrontRight.getDeviceID());
        } else {
            //TODO write velocity control mode outputs

            driveFrontLeft.set(ControlMode.Velocity, periodic.left_demand, DemandType.ArbitraryFeedForward,
                    (periodic.left_feedforward + Constants.DRIVE_LEFT_KD* periodic.left_accl / 1023.0));
            driveFrontRight.set(ControlMode.Velocity, periodic.right_demand, DemandType.ArbitraryFeedForward,
                    -(periodic.right_feedforward + Constants.DRIVE_RIGHT_KD * periodic.right_accl / 1023.0));
            // driveFrontLeft.set(ControlMode.Velocity, periodic.left_demand, DemandType.ArbitraryFeedForward, 0.0);
            driveMiddleLeft.set(ControlMode.Follower, driveFrontLeft.getDeviceID());
            driveBackLeft.set(ControlMode.Follower, driveFrontLeft.getDeviceID());
            //driveFrontRight.set(ControlMode.Velocity, periodic.right_demand, DemandType.ArbitraryFeedForward, 0.0);
            driveMiddleRight.set(ControlMode.Follower, driveFrontRight.getDeviceID());
            driveBackRight.set(ControlMode.Follower, driveFrontRight.getDeviceID());
        }
        if (periodic.B2) {
            trans.set(DoubleSolenoid.Value.kForward);
        } else {
            trans.set(DoubleSolenoid.Value.kReverse);
        }

    }


    public void outputTelemetry() {
        SmartDashboard.putNumber("SmartDashboard/Drive/Right", periodic.right_pos_ticks);
        SmartDashboard.putNumber("Left", periodic.left_pos_ticks);
        SmartDashboard.putNumber("Heading", pigeonIMU.getCompassHeading());
        SmartDashboard.putString("Drive State", mDriveControlState.toString());
        SmartDashboard.putNumberArray("drivedemands", new double[]{periodic.left_demand, periodic.right_demand});
        SmartDashboard.putNumberArray("drivevels", new double[]{periodic.left_velocity_ticks_per_100ms, periodic.right_velocity_ticks_per_100ms});
        SmartDashboard.putNumber("Robot Error X", periodic.error.getTranslation().x());
        SmartDashboard.putNumber("Robot Error Y", periodic.error.getTranslation().y());
        SmartDashboard.putNumber("Robot Error Theta", periodic.error.getRotation().getDegrees());
        SmartDashboard.putNumber("Robot Setpoint X", periodic.path_setpoint.state().getTranslation().x());
        SmartDashboard.putNumber("Robot Setpoint Y", periodic.path_setpoint.state().getTranslation().y());
        SmartDashboard.putNumber("Variables/Robot Setpoint Theta", periodic.path_setpoint.state().getRotation().getDegrees());
        SmartDashboard.putNumber("Left Talon Velocity", periodic.left_velocity_ticks_per_100ms);
        SmartDashboard.putNumber("Right Talon Velocity", periodic.right_velocity_ticks_per_100ms);
        SmartDashboard.putNumber("Right Talon Error", driveFrontRight.getClosedLoopError(0));
        SmartDashboard.putNumber("Left Talon Error", driveFrontLeft.getClosedLoopError(0));
        SmartDashboard.putNumber("Right Talon Voltage", driveFrontRight.getBusVoltage());
        SmartDashboard.putNumber("Left Talon Voltage", driveFrontLeft.getBusVoltage());
        SmartDashboard.putNumber("Right Talon Voltage II", driveFrontRight.getMotorOutputVoltage());
        SmartDashboard.putNumber("Left Talon Voltage II", driveFrontLeft.getMotorOutputVoltage());
        SmartDashboard.putNumber("Right Encoder Counts", periodic.right_pos_ticks);
        SmartDashboard.putNumber("Left Encoder Counts", periodic.left_pos_ticks);
        SmartDashboard.putNumber("Left FeedForward", periodic.left_feedforward);
        SmartDashboard.putNumber("Left Acceleration", periodic.left_accl);
        SmartDashboard.putNumber("Right FeedForward", periodic.right_feedforward);
        SmartDashboard.putNumber("Right Acceleration", periodic.right_accl);
        SmartDashboard.setDefaultNumber("Left Talon Voltage", 10);
    }



    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    @Override
    public void stop() {

    }

    enum DriveControlState {
        OPEN_LOOP,
        PATH_FOLLOWING,
        PROFILING_TEST;

        @Override
        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    public static class PeriodicIO {
        // INPUTS
        public int left_pos_ticks;
        public int left_velocity_ticks_per_100ms;
        public int right_pos_ticks;
        public int right_velocity_ticks_per_100ms;
        public Rotation2d gyro_heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();
        public boolean B2 = false;


        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double right_distance;
        public double left_distance;
        public double left_feedforward;
        public double right_feedforward;
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<>(Pose2dWithCurvature.identity());
        public double left_accl;
        public double right_accl;
    }
    private void gearShift(){
        if (getLeftLinearVelocity() >= 10) {
            periodic.B2 = true;
        }
        if (getLeftLinearVelocity() <= 10) {
            periodic.B2 = false;
        }
    }
}
