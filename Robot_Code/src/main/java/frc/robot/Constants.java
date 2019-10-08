package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.lib.util.HIDHelper;

public class Constants {

    /**
     * device ID declarations ---------------------------------
     */

    //Talon IDs
    public static final int DRIVE_FRONT_LEFT_ID = 4;
    public static final int DRIVE_MIDDLE_LEFT_ID = 5;
    public static final int DRIVE_BACK_LEFT_ID = 6;
    public static final int DRIVE_FRONT_RIGHT_ID = 1;
    public static final int DRIVE_MIDDLE_RIGHT_ID = 2;
    public static final int DRIVE_BACK_RIGHT_ID = 3;
    //public static final int ARM_PRONOMINAL = 7;
    public static final int ARM_DISTAL = 8;

    //Spark Ports
    public static final int BOTTOM_CARGOMANIP_ID = 1;
    public static final int TOP_CARGOMANIP_ID = 2;
    public static final int CLIMBER_CRAWL_ID = 3;
    public static final int CLIMBER_ELEVATOR_ID = 4;

    //Solenoid Ports
    public static final int TRANS_LOW_ID = 0;
    public static final int TRANS_HIGH_ID = 1;
    public static final int CLIMB_FRONT_LOW_ID = 2;
    public static final int CLIMB_FRONT_HIGH_ID = 3;
    public static final int LOCK_LOW_ID = 4;
    public static final int LOCK_HIGH_ID = 5;
    public static final int PROX_LOW = 6;
    public static final int PROX_HIGH = 7;

    //Sensor Ports
    //public static final int ULTRASONIC_IN_1 = 0;
    //public static final int ULTRASONIC_OUT_1 = 1;
    //public static final int ULTRASONIC_IN_2 = 2;
    //public static final int ULTRASONIC_OUT_2 = 3;

    /**
     * Drivetrain tuned values --------------------------------
     */

    //Physical Constants
    public static final double DRIVE_WHEEL_TRACK_WIDTH_INCHES = 21.75;
    public static final double DRIVE_WHEEL_DIAMETER_INCHES = 6.225; // 6
    public static final double DRIVE_WHEEL_RADIUS_INCHES = DRIVE_WHEEL_DIAMETER_INCHES / 2.0;
    public static final double TRACK_SCRUB_FACTOR = 1.0;  // TODO tune
    public static final double ROBOT_LINEAR_INERTIA = 75;  // kg TODO tune
    public static final double ROBOT_ANGULAR_INERTIA = 10.0;  // kg m^2 TODO tune
    public static final double ROBOT_ANGULAR_DRAG = 12.0;  // N*m / (rad/sec) TODO tune

    //Path following Constants
    public static final double ROBOT_MAX_VELOCITY = 120.0; // in/s
    public static final double ROBOT_MAX_ACCEL = 120.0; // in/s^2
    public static final double ROBOT_MAX_VOLTAGE = 10.0; // V
    public static final double Path_Kx = 4.0;  //
    public static final double PATH_LOOK_AHEAD_TIME = 0.4;  // seconds to look ahead along the path for steering
    public static final double PATH_MIN_LOOK_AHEAD_DISTANCE = 24.0;  // inches

    //Electrical Constants
    public static final double DRIVE_V_INTERCEPT = 1.2;  // V //1.6 for practice......................
    public static final double DRIVE_Kv = 0.316426;  // V per rad/s -.335
    public static final double DRIVE_Ka = 0.0801;  // V per rad/s^2    0.0801
    public static final double DRIVE_VCOMP = 10.0; //V
    public static final double DRIVE_ENCODER_PPR = 4096.0; //encoder counts per revolution

    //PID Constants
    public static final double ANGLE_KP = 0.04; // 0.065;
    public static final double ANGLE_KI = 0; // 0.00125;
    public static final double ANGLE_KD = 0; // 0.1

    public static final double DRIVE_RIGHT_KP = 1.2;
    public static final double DRIVE_RIGHT_KI = 0.0;
    public static final double DRIVE_RIGHT_KD = 25; // 20 for practice bot
    public static final double DRIVE_RIGHT_KF = 0.53; //.485

    public static final double DRIVE_LEFT_KP = 1.1; // .0885
    public static final double DRIVE_LEFT_KI = 0.0; //NO INTEGRAL it masks deeper problems
    public static final double DRIVE_LEFT_KD = 25; //20 for practice
    public static final double DRIVE_LEFT_KF = 0.53;


    /**
     * Arm tuned values -----------------------------------------
     */

    // Arm Absolute Zeros
    //TODO CHECK TWICE BONEHEAD!!!!!!!
    public static double DIST_ABSOLUTE_ZERO = 2262;//8293

    //Arm Physical Constants
    public static final double PROX_LENGTH = 0.6096; // m
    public static final double DIST_LENGTH = 0.4572; // m
    public static final double PROX_MOI = 0.1;
    public static final double DIST_MOI = 0.01;

    //Electrical Constants
    public static final double PROX_Kv = 0.1; // V per rad/s
    public static final double PROX_Kt = 0.5; // V per rad/s^2 //not currently correct usage
    public static final double PROX_V_INTERCEPT = 0.1; // V

    public static final double DIST_Kv = 0.1; // V per rad/s
    public static final double DIST_Kt = 0.5; // V per rad/s^2 //not currently correct usage
    public static final double DIST_V_INTERCEPT = 0.1; // V

    public static final double ARM_DIST_KP = 3.5;//7.5
    public static final double ARM_DIST_KI = 0;
    public static final double ARM_DIST_KD = 0;//75
    public static final double ARM_DIST_KF = 0;
    //public static final double ARM_DIST_A_FEEDFORWARD = 0; //.16
    public static final double ARM_U_U_LIMIT = 750;
    public static final double ARM_U_L_LIMIT = -1500;
    public static final double ARM_L_U_LIMIT = 650;
    public static final double ARM_L_L_LIMIT = -350;
    public static final double ARM_NO_DOWN_LIMIT = -500;
    /**
     * General Configuration --------------------------------------
     */

    //UltraSonic constants
    //public static final double US_UPDATE_RATE = 1.2;
    //public static final double US_SENSOR_OFFSET = 10;

    //Shot powers
    public static final double SHOOT_POWER = 1;
    public static final double SLOW_SHOOT_POWER = .5;
    public static final double PICKUP_POWER = -.55;
    public static final double SLOWUP_POWER = -.35;
    public static final double STOP_POWER = 0;
    public static final double CLIMB_POWER = .25;

    //MP Test mode values
    public static final boolean ENABLE_MP_TEST_MODE = true; //enables motion profiling test across all modes
    public static final double MP_TEST_SPEED = 72; //in /s
    public static final boolean RAMPUP = false;

    //Stick Constants
    public static final Joystick MASTER = new Joystick(0);
    public static final Joystick LAUNCH_PAD = new Joystick(1);
    public static final HIDHelper.HIDConstants MASTER_STICK = new HIDHelper.HIDConstants(MASTER, 0.2, 0.99, 0.99, 0.6, 2);
    public static final HIDHelper.HIDConstants LAUNCHPAD_STICK = new HIDHelper.HIDConstants(LAUNCH_PAD, 0.2, 0.99, 0.99, 0.8, 2);

    //Startup Constants
    public static final double LOOPER_DT = 0.01; //dt in seconds
    public static final boolean IS_COMP_BOT = true;
    public static final String DATA_SEPARATOR = ",";
    public static final String[] NUMBER_KEYS = {
            "Drive/Pose/Theta", //2
            "Drive/Pose/X", //3
            "Drive/Pose/Y", //4
            "Drive/Error/Theta", //5
            "Drive/Error/X", //6
            "Drive/Error/Y", //7
            "Drive/Setpoint/Theta", //8
            "Drive/Setpoint/X", //9
            "Drive/Setpoint/Y", //10
            "Drive/Left/Demand", //11
            "Drive/Right/Demand", //12
            "Drive/Left/Talon Velocity", //13
            "Drive/Right/Talon Velocity", //14
            "Drive/Error/Left Talon Error", //15
            "Drive/Error/Right Talon Error", //16
            "Drive/Misc/Left FeedForward", //17
            "Drive/Misc/Right FeedForward", //18
            "Drive/Left/Talon Voltage Out", //19
            "Drive/Right/Talon Voltage Out", //20
            "Arm/Dist Point", // 21
            "Vision/rioStatus", //22
            "StateMachine/state" //23
    };
}



