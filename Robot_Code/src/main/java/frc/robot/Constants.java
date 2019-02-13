package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.lib.util.HIDHelper;
import frc.robot.subsystems.Arm;

public class Constants {
    //Talon IDs
    public static final int DRIVE_FRONT_LEFT_ID = 3;
    public static final int DRIVE_MIDDLE_LEFT_ID = 2;
    public static final int DRIVE_BACK_LEFT_ID = 1;
    public static final int DRIVE_FRONT_RIGHT_ID = 6;
    public static final int DRIVE_MIDDLE_RIGHT_ID = 5;
    public static final int DRIVE_BACK_RIGHT_ID = 4;
    public static final int ARM_PRONOMINAL = 7;
    public static final int ARM_DISTAL = 8;
    public static final int ARM_WRIST = 9;

    //Spark Ports
    public static final int TOP_CARGOMANIP_ID = 1;
    public static final int BOTTOM_CARGOMANIP_ID = 2;
    public static final int LEFT_CLIMB_ID = 3;
    public static final int RIGHT_CLIMB_ID = 4;

    //Solenoid Ports
    public static final int TRANS_LOW_ID = 0;
    public static final int TRANS_HIGH_ID = 1;
    public static final int ALIEN_1_LOW_ID = 2;
    public static final int ALIEN_1_HIGH_ID = 3;

    //Sensor Ports
    public static final int ULTRASONIC_IN_1 = 0;
    public static final int ULTRASONIC_OUT_1 = 1;
    public static final int ULTRASONIC_IN_2 = 2;
    public static final int ULTRASONIC_OUT_2 = 3;

    //Pure pursuit related values
    public static final double DRIVE_WHEEL_TRACK_WIDTH_INCHES = 23;
    public static final double DRIVE_WHEEL_DIAMETER_INCHES = 6.225;
    public static final double DRIVE_WHEEL_RADIUS_INCHES = DRIVE_WHEEL_DIAMETER_INCHES / 2.0;
    public static final double TRACK_SCRUB_FACTOR = 1.0;  // Tune me!
    public static final double ROBOT_LINEAR_INERTIA = 59;  // kg TODO tune
    public static final double ROBOT_ANGULAR_INERTIA = 10.0;  // kg m^2 TODO tune
    public static final double ROBOT_ANGULAR_DRAG = 12.0;  // N*m / (rad/sec) TODO tune
    public static final double ROBOT_MAX_VELOCITY = 120.0; // TODO tune & find units
    public static final double ROBOT_MAX_ACCEL = 120.0; // TODO tune & find units
    public static final double ROBOT_MAX_VOLTAGE = 10.0; // V
    public static final double DRIVE_V_INTERCEPT = 1.4;  // V
    public static final double DRIVE_Kv = 0.20;  // V per rad/s 0.5234375 //TODO REDETERMINE
    public static final double DRIVE_Ka = 0.1;  // V per rad/s^2
    public static final double Path_Kx = 4.0;  //
    public static final double DRIVE_VCOMP = 10.0; //V
    public static final double PATH_LOOK_AHEAD_TIME = 0.4;  // seconds to look ahead along the path for steering
    public static final double PATH_MIN_LOOK_AHEAD_DISTANCE = 24.0;  // inches
    public static final double DRIVE_ENCODER_PPR = 4096.0; //encoder counts per revolution

    // Arm Absolute Zeros
    public static final double ProxAbsoluteZero = 3532;
    public static final double DistAbsoluteZero = 2733;
    public static final double WristAbsoluteZero = 515;

    //Arm Configurations
    public static final Arm.ArmConfiguration FWD_GROUND_HATCH = new Arm.ArmConfiguration(0,0,0);
    public static final Arm.ArmConfiguration FWD_GROUND_CARGO = new Arm.ArmConfiguration(0,0,0);
    public static final Arm.ArmConfiguration FWD_LOW_HATCH = new Arm.ArmConfiguration(0,0,0);
    public static final Arm.ArmConfiguration FWD_LOW_CARGO = new Arm.ArmConfiguration(0,0,0);
    public static final Arm.ArmConfiguration FWD_MEDIUM_HATCH = new Arm.ArmConfiguration(0,0,0);
    public static final Arm.ArmConfiguration FWD_MEDUIM_CARGO = new Arm.ArmConfiguration(0,0,0);
    public static final Arm.ArmConfiguration FWD_HIGH_HATCH = new Arm.ArmConfiguration(0,0,0);
    public static final Arm.ArmConfiguration FWD_HIGH_CARGO = new Arm.ArmConfiguration(0,0,0);

    public static final Arm.ArmConfiguration STRAIGHT_UP = new Arm.ArmConfiguration(0,0,0);

    public static final Arm.ArmConfiguration REV_GROUND_HATCH = new Arm.ArmConfiguration(0,0,0);
    public static final Arm.ArmConfiguration REV_GROUND_CARGO = new Arm.ArmConfiguration(0,0,0);
    public static final Arm.ArmConfiguration REV_LOW_HATCH = new Arm.ArmConfiguration(0,0,0);
    public static final Arm.ArmConfiguration REV_LOW_CARGO = new Arm.ArmConfiguration(0,0,0);
    public static final Arm.ArmConfiguration REV_MEDIUM_HATCH = new Arm.ArmConfiguration(0,0,0);
    public static final Arm.ArmConfiguration REV_MEDUIM_CARGO = new Arm.ArmConfiguration(0,0,0);
    public static final Arm.ArmConfiguration REV_HIGH_HATCH = new Arm.ArmConfiguration(0,0,0);
    public static final Arm.ArmConfiguration REV_HIGH_CARGO = new Arm.ArmConfiguration(0,0,0);

    //Shot powers
    public static final double SHOOT_POWER = 1;
    public static final double ROLLOUT_POWER = .9;
    public static final double DROP_POWER = .65;
    public static final double PICKUP_POWER = -.75;
    public static final double SLOWUP_POWER = -.65;
    public static final double STOP_POWER = 0;

    //PID Gain Constants
    public static final double DRIVE_RIGHT_KP = 1.2;
    public static final double DRIVE_RIGHT_KI = 0.0;
    public static final double DRIVE_RIGHT_KD = 20;
    public static final double DRIVE_RIGHT_KF = 0.53; //.485

    public static final double DRIVE_LEFT_KP = 1.1; // .0885
    public static final double DRIVE_LEFT_KI = 0.0; //NO INTEGRAL it masks deeper problems
    public static final double DRIVE_LEFT_KD = 25; //10
    public static final double DRIVE_LEFT_KF = 0.53;

    public static final double ARM_PROX_KP = 0.01;
    public static final double ARM_PROX_KI = 0;
    public static final double ARM_PROX_KD = 0;
    public static final double ARM_PROX_KF = 0;
    public static final double ARM_PROX_A_FEEDFORWARD = 1;
    public static final double ARM_DIST_A_FEEDFORWARD = 1;
    public static final double ARM_WRIST_A_FEEDFORWARD = 1;
    public static final double ARM_DIST_KP = 0.01;
    public static final double ARM_DIST_KI = 0;
    public static final double ARM_DIST_KD = 0;
    public static final double ARM_DIST_KF = 0;
    public static final double ARM_WRIST_KP = 0.01;
    public static final double ARM_WRIST_KI = 0;
    public static final double ARM_WRIST_KD = 0;
    public static final double ARM_WRIST_KF = 0;


    public static final double ANGLE_KP = 0.0425;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.015;
    //UltraSonic constants
    public static final double US_UPDATE_RATE = 240 * .005;
    public static final double US_SENSOR_OFFSET = 3;

    //MP Test mode values
    public static final boolean ENABLE_MP_TEST_MODE = true; //enables motion profiling test across all modes
    public static final double MP_TEST_SPEED = 72; //in /s
    public static final boolean RAMPUP = false;

    //Stick Constants
    public static final Joystick MASTER = new Joystick(0);
    public static final Joystick LAUNCH_PAD = new Joystick(1);
    public static final HIDHelper.HIDConstants MASTER_STICK = new HIDHelper.HIDConstants(MASTER, 0.15, 1.0, 1.0, 0.6, 2);

    //Startup Constants
    public static final double LOOPER_DT = 0.01; //dt in seconds
    public static final boolean IS_COMP_BOT = true;
    public static final String ROBOT_NAME = "Whatever_you_want";
    public static final String DATA_SEPARATOR = ",";
    public static final String[] NUMBER_KEYS = {
            "Drive/Pose/Theta",
            "Drive/Pose/X",
            "Drive/Pose/Y",
            "Drive/Error/Theta",
            "Drive/Error/X",
            "Drive/Error/Y",
            "Drive/Setpoint/Theta",
            "Drive/Setpoint/X",
            "Drive/Setpoint/Y",
            "Drive/Left Demand",
            "Drive/Right Demand",
            "Drive/Left Talon Velocity",
            "Drive/Right Talon Velocity",
            "Drive/Error/Left Talon Error",
            "Drive/Error/Right Talon Error",
            "Drive/Misc/Left FeedForward",
            "Drive/Misc/Right FeedForward"
    };
}



