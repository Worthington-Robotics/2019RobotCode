package frc.robot.planners;

public class ArmMotionPlanner {
    private double m_A, m_V, m_T;

    public ArmMotionPlanner() {
        //put model here?
    }

    public double update() {

        return 0;

    }

    public static class Output{

        public double proxVel = 0.0, proxAccel = 0.0, proxArbFF = 0.0;
        public double distVel = 0.0, distAccel = 0.0, distArbFF = 0.0;

        public Output(double proxVel, double proxAccel, double proxArbFF, double distVel, double distAccel, double distArbFF){
            this.proxVel = proxVel;
            this.proxAccel = proxAccel;
            this.proxArbFF = proxArbFF;
            this.distVel = distVel;
            this.distAccel = distAccel;
            this.distArbFF = distArbFF;
        }

    }

    enum FollowerType{
        TRAPEZOIDAL,
        PID_MODEL,
        FEED_FORWARD_ONLY
    }
}