package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.lib.util.HIDHelper;

public class Constants {
    //Talon IDs
    public static final int DRIVE_FRONT_LEFT_ID = 1;
    public static final int DRIVE_MIDDLE_LEFT_ID = 2;
    public static final int DRIVE_BACK_LEFT_ID = 3;
    public static final int DRIVE_FRONT_RIGHT_ID = 4;
    public static final int DRIVE_MIDDLE_RIGHT_ID = 5;
    public static final int DRIVE_BACK_RIGHT_ID = 6;
    public static final int ARM_PROXIMINAL = 7;
    public static final int ARM_DISTAL = 8;
    public static final int ARM_WRIST = 9;

    //Spark Ports
    public static final int TOP_CARGOMANIP_ID = 1;
    public static final int BOTTOM_CARGOMANIP_ID = 2;


    //Solenoid Ports
    public static final int TRANS_LOW_ID = 0;
    public static final int TRANS_HIGH_ID = 1;
    public static final int ALIEN_1_LOW_ID = 2;
    public static final int ALIEN_1_HIGH_ID = 3;


    //Pure pursuit related values
    public static final double DRIVE_WHEEL_TRACK_WIDTH_INCHES = 23.54;
    public static final double DRIVE_WHEEL_DIAMETER_INCHES = 6.5;
    public static final double DRIVE_WHEEL_RADIUS_INCHES = DRIVE_WHEEL_DIAMETER_INCHES / 2.0;
    public static final double TRACK_SCRUB_FACTOR = 1.0;  // Tune me!
    public static final double ROBOT_LINEAR_INERTIA = 66.5;  // kg TODO tune
    public static final double ROBOT_ANGULAR_INERTIA = 10.0;  // kg m^2 TODO tune
    public static final double ROBOT_ANGULAR_DRAG = 12.0;  // N*m / (rad/sec) TODO tune
    public static final double ROBOT_MAX_VELOCITY = 120.0; // TODO tune & find units
    public static final double ROBOT_MAX_ACCEL = 120.0; // TODO tune & find units
    public static final double ROBOT_MAX_VOLTAGE = 10.0; // V TODO tune
    public static final double DRIVE_V_INTERCEPT = 1.1;  // V
    public static final double DRIVE_Kv = 0.584;  // V per rad/s
    public static final double DRIVE_Ka = 0.1;  // V per rad/s^2
    public static final double Path_Kx = 4.0;  //
    // units/s per unit of error
    public static final double PATH_LOOK_AHEAD_TIME = 0.4;  // seconds to look ahead along the path for steering
    public static final double PATH_MIN_LOOK_AHEAD_DISTANCE = 24.0;  // inches
    public static final double DRIVE_ENCODER_PPR = 4096.0; //encoder counts per revolution
    public static final double TICKS_TO_INCHES = 1625;
    public static final double ROTATIONS_TO_INCHES = TICKS_TO_INCHES/DRIVE_ENCODER_PPR;

    //Shot powers
    public static final double SHOOT_POWER = 1;
    public static final double ROLLOUT_POWER = .9;
    public static final double DROP_POWER = .65;
    public static final double PICKUP_POWER = -.75;
    public static final double SLOWUP_POWER = -.65;
    public static final double STOP_POWER = 0;

    //Alien Constants

    //PID Gain Constants
    public static final double DRIVE_RIGHT_KP = 0.0885;
    public static final double DRIVE_RIGHT_KI = 0;
    public static final double DRIVE_RIGHT_KD = 6;
    public static final double DRIVE_RIGHT_KF = .066;
    public static final double DRIVE_LEFT_KP = 0.0885;
    public static final double DRIVE_LEFT_KI = 0;
    public static final double DRIVE_LEFT_KD = 6;
    public static final double DRIVE_LEFT_KF = .066;
    public static final double ARM_PROX_KP = 0.01;
    public static final double ARM_PROX_KI = 0;
    public static final double ARM_PROX_KD = 0;
    public static final double ARM_PROX_KF = 0;
    public static final double ARM_DIST_KP = 0.01;
    public static final double ARM_DIST_KI = 0;
    public static final double ARM_DIST_KD = 0;
    public static final double ARM_DIST_KF = 0;
    public static final double ARM_WRIST_KP = 0.01;
    public static final double ARM_WRIST_KI = 0;
    public static final double ARM_WRIST_KD = 0;
    public static final double ARM_WRIST_KF = 0;


    //Update times / rates / logger constants
    public static final double LOOPER_DT = 0.01; //dt in seconds
    public static final double LOGGING_UPDATE_RATE = .02;

    public static final String DATA_SEPARATOR = ",";


    //MP Test mode values
    public static final boolean ENABLE_MP_TEST_MODE = true; //enables motion profiling test across all modes
    public static final double MP_TEST_SPEED = 4;
    public static final boolean RAMPUP = false;

    //Stick Constants
    public static final Joystick MASTER = new Joystick(0);
    public static final Joystick SECOND = new Joystick(1);
    public static final HIDHelper.HIDConstants MASTER_STICK = new HIDHelper.HIDConstants(MASTER, 0.15, 1.0, 1.0, 0.45, 2);
    public static final HIDHelper.HIDConstants SECOND_STICK = new HIDHelper.HIDConstants(SECOND, 0.05, 1.0, 1.0, 1.0, 2);

    //Startup Constants
    public static final boolean IS_COMP_BOT = true;
    public static final String ROBOT_NAME = "Whatever_you_want";

}



