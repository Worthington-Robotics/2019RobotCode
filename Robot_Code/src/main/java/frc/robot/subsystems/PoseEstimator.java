package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Twist2d;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.InterpolatingDouble;
import frc.lib.util.InterpolatingTreeMap;
import frc.robot.Kinematics;

import java.util.Map;


public class PoseEstimator extends Subsystem {

    private static PoseEstimator m_instance = new PoseEstimator();

    private static final int observation_buffer_size_ = 100;

    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private double left_encoder_prev_distance_ = 0.0;
    private double right_encoder_prev_distance_ = 0.0;
    private double distance_driven_= 0.0;

    private Loop mLoop = new Loop(){

        @Override
        public void onStart(double timestamp) {
            distance_driven_= 0.0;
            left_encoder_prev_distance_ = Drive.getInstance().getLeftEncoderDistance();
            right_encoder_prev_distance_ = Drive.getInstance().getRightEncoderDistance();
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (this){
                final Rotation2d gyro_angle = Drive.getInstance().getHeading();
                final double left_distance = Drive.getInstance().getLeftEncoderDistance();
                final double right_distance = Drive.getInstance().getRightEncoderDistance();
                final double delta_left = left_distance - left_encoder_prev_distance_;
                final double delta_right = right_distance - right_encoder_prev_distance_;
                final Twist2d odometry_velocity = generateOdometryFromSensors(delta_left, delta_right, gyro_angle);
                addObservations(timestamp, Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), odometry_velocity));
                left_encoder_prev_distance_ = left_distance;
                right_encoder_prev_distance_ = right_distance;
                outputTelemetry();
            }
        }

        @Override
        public void onStop(double timestamp) {

        }
    };

    public static PoseEstimator getInstance(){
        return m_instance;
    }

    private PoseEstimator(){
        reset(0, Pose2d.identity());
    }

    public void reset(double start_time, Pose2d initial_field_to_vehicle){
        field_to_vehicle_ = new InterpolatingTreeMap<>(observation_buffer_size_);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        distance_driven_ = 0.0;
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Twist2d generateOdometryFromSensors(double left_encoder_delta_distance, double
            right_encoder_delta_distance, Rotation2d current_gyro_angle) {
        final Pose2d last_measurement = getLatestFieldToVehicle().getValue();
        final Twist2d delta = Kinematics.forwardKinematics(last_measurement.getRotation(),
                left_encoder_delta_distance, right_encoder_delta_distance,
                current_gyro_angle);
        distance_driven_ += delta.dx; //do we care about dy here?
        return delta;
    }

    public synchronized void addObservations(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public double getDistanceDriven(){
        return distance_driven_;
    }

    @Override
    public void outputTelemetry() {
        Pose2d odometry = getLatestFieldToVehicle().getValue();
        SmartDashboard.putNumber("Drive/Pose/X", odometry.getTranslation().x());
        SmartDashboard.putNumber("Drive/Pose/Y", odometry.getTranslation().y());
        SmartDashboard.putNumber("Drive/Pose/Theta", (odometry.getRotation().getDegrees()+360)%360);
    }

    @Override
    public void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }

    @Override
    public void registerEnabledLoops(ILooper looper){
        looper.register(mLoop);
    }
}
