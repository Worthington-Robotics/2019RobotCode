package frc.lib.physics;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.util.CSVWritable;
import frc.robot.Constants;

import java.text.DecimalFormat;

public class ArmModel {

    // Equivalent moment of inertia when accelerating purely angularly, in kg*m^2.
    protected final double prox_moi_;
    protected final double dist_moi_;

    // Equivalent mass of segment, in kg.


    // Transmissions for both joints of the arm.
    protected final DCMotorTransmission prox_transmission_;
    protected final DCMotorTransmission dist_transmission_;


    public ArmModel(double prox_moi, double dist_moi, DCMotorTransmission prox_transmission, DCMotorTransmission dist_transmission) {
        this.prox_moi_ = prox_moi;
        this.dist_moi_ = dist_moi;
        this.prox_transmission_ = prox_transmission;
        this.dist_transmission_ = dist_transmission;
    }

    //TODO implement actual model

    /**
     * Generates an X Y coordinate for the end effector based on two angles
     * <p>Can be angular position, velocity or acceleration
     *
     * @param prox
     * @param dist
     * @return double array of length 2 {x, y}
     */
    public static Translation2d solveForwardKinematics(Rotation2d prox, Rotation2d dist) {
        double x = (prox.sin() * Constants.PROX_LENGTH) + (dist.sin() * Constants.DIST_LENGTH); // Domains are reversed due to 0 being perfectly vertical
        double y = (prox.sin() * Constants.PROX_LENGTH) + (dist.cos() * Constants.DIST_LENGTH);
        return new Translation2d(x, y);
    }


    /**
     * Generates a set of angles for an xy coordinate
     * <p>Can be position, velocity or acceleration
     *
     * @param x X coordinate in m
     * @param y Y coordinate in m
     * @return a position based arm state
     */
    public static ArmState solveInverseKinematics(double x, double y) {
        //TODO determine if domain shift is necessary here
        double distTheta = Math.acos((x * x + y * y
                - Constants.PROX_LENGTH * Constants.PROX_LENGTH - Constants.DIST_LENGTH * Constants.DIST_LENGTH) / (2 * Constants.DIST_LENGTH * Constants.PROX_LENGTH));
        double proxTheta = Math.atan2(y, x) - Math.atan2(Constants.DIST_LENGTH * Math.sin(distTheta),
                Constants.PROX_LENGTH + Constants.DIST_LENGTH * Math.cos(distTheta));
        distTheta += proxTheta;
        return new ArmState(proxTheta, distTheta);
    }

    public ArmDynamics solveForwardDynamics(ArmState angularPosition, ArmState Voltage, ArmState AngularVelocity) {
        double proxGravityTorque = 9.81 * Math.sin(angularPosition.get(true)) * Constants.PROX_LENGTH * prox_moi_;
        double distGravityTorque = 9.81 * Math.sin(angularPosition.get(false)) * Constants.DIST_LENGTH * dist_moi_;
        ArmDynamics output = new ArmDynamics();
        output.angular_velocity = AngularVelocity;
        output.voltage = Voltage;
        double proxTorque = prox_transmission_.getTorqueForVoltage(AngularVelocity.get(true), Voltage.get(true)) - proxGravityTorque;
        double distTorque = dist_transmission_.getTorqueForVoltage(AngularVelocity.get(false), Voltage.get(false)) - distGravityTorque;
        ArmState Torque = new ArmState();
        Torque.set(true, proxTorque);
        Torque.set(false, distTorque);
        output.torque = Torque;
        double proxAngularAcceleration = proxTorque / prox_moi_;
        double distAngularAcceleration = distTorque / dist_moi_;
        ArmState AngularAcceleration = new ArmState();
        AngularAcceleration.set(true, proxAngularAcceleration);
        AngularAcceleration.set(false, distAngularAcceleration);
        output.angular_acceleration = AngularAcceleration;
        return output;
    }

    public ArmDynamics solveInverseDynamics(ArmState angularPosition, ArmState angularVelocity, ArmState angularAcceleration) {
        double proxGravityTorque = 9.81 * Math.sin(angularPosition.get(true)) * Constants.PROX_LENGTH * prox_moi_;
        double distGravityTorque = 9.81 * Math.sin(angularPosition.get(false)) * Constants.DIST_LENGTH * dist_moi_;
        ArmDynamics output = new ArmDynamics();
        output.angular_acceleration = angularAcceleration;
        output.angular_velocity = angularVelocity;
        double proxTorque = angularAcceleration.get(true) * prox_moi_ - proxGravityTorque;
        double distTorque = angularAcceleration.get(false) * dist_moi_ - distGravityTorque;
        ArmState Torque = new ArmState();
        Torque.set(true, proxTorque);
        Torque.set(false, distTorque);
        output.torque = Torque;
        double proxVoltage = prox_transmission_.getVoltageForTorque(angularVelocity.get(true), Torque.get(true));
        double distVoltage = dist_transmission_.getVoltageForTorque(angularVelocity.get(false), Torque.get(false));
        ArmState Voltage = new ArmState();
        Voltage.set(true, proxVoltage);
        Voltage.set(false, distVoltage);
        output.voltage = Voltage;
        return output;

    }

    // Can refer to velocity, acceleration, torque, voltage, etc., depending on context.
    public static class ArmState {
        public double prox = 0.0;
        public double dist = 0.0;

        public ArmState(double prox, double dist) {
            this.prox = prox;
            this.dist = dist;
        }

        public ArmState() {
        }

        public double get(boolean get_prox) {
            return get_prox ? prox : dist;
        }

        public void set(boolean set_prox, double val) {
            if (set_prox) {
                prox = val;
            } else {
                dist = val;
            }
        }

        @Override
        public String toString() {
            DecimalFormat fmt = new DecimalFormat("#0.000");
            return fmt.format(prox) + ", " + fmt.format(dist);
        }

        public boolean isNAN(){
            return prox == Double.NaN || dist == Double.NaN;
        }
    }

    //full state dynamics of a 2 axis arm
    public static class ArmDynamics implements CSVWritable {
        public ArmState angular_velocity = new ArmState(); // rad/s
        public ArmState angular_acceleration = new ArmState(); // rad/s^2
        public ArmState voltage = new ArmState(); // V
        public ArmState torque = new ArmState(); // N m

        public String toCSV() {
            return angular_velocity + ", " + angular_acceleration + ", " + voltage + ", " + torque;
        }

        @Override
        public String toString() {
            return "omega:(" + angular_velocity + ") alpha:(" + angular_acceleration + ") V:(" + voltage + ") tau:(" + torque + ")";
        }
    }

}
