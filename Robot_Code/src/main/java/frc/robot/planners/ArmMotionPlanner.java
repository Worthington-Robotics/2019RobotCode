package frc.robot.planners;

public class ArmMotionPlanner {
    private double m_A, m_V, m_T;

    public ArmMotionPlanner() {
        //put model here?
    }

    /**
     * A method that returns a Voltage based on a current velocity and acceleration
     * @param A current acceleration
     * @param V current Velocity
     * @param theta1 Start theta
     * @param theta2 End theta
     * @return Voltage required
     */
    public double MotionPlanner(double A, double V, double theta1, double theta2) {
        double deltaTheta = theta2 - theta1;
        return 0;

    }
}