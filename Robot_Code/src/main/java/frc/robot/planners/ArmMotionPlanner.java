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
     * @param deltaTheta change in theta desired
     * @return Voltage required
     */
    public double MotionPlanner(double A, double V, double deltaTheta) {

    }
}