package frc.robot.subsystems.vision;

import frc.robot.subsystems.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends Subsystem {

    public static final String NOT_INIT = "NOT_INIT";
    public static final String PING = "PING";
    public static final String PONG = "PONG";
    public static final String CONNECTED = "CONNECTED";
    public static final String RESET = "RESET";
    public static final String TABLE_NAME = "vision/connect";

    // construct one and only 1 instance of this class
    private static Vision m_VisionInstance = new Vision();

    private String connectionStatus = NOT_INIT;

    public static Vision getInstance() {
        return m_VisionInstance;
    }

    private Vision() {

    }

    @Override
    public synchronized void readPeriodicInputs() {
        SmartDashboard.clearPersistent(TABLE_NAME);

        String status = SmartDashboard.getString(TABLE_NAME, NOT_INIT);

        switch (status) {
        case NOT_INIT:
            this.connectionStatus = NOT_INIT;
            break;
        case PONG:
            this.connectionStatus = PONG;
            break;
        case RESET:
            this.connectionStatus = NOT_INIT;
            break;
        default:
            break;
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {

        switch (this.connectionStatus) {
        case NOT_INIT:
            SmartDashboard.putString(TABLE_NAME, PING);
            this.connectionStatus = PING;
            break;
        case PONG:
            SmartDashboard.putString(TABLE_NAME, CONNECTED);
            this.connectionStatus = CONNECTED;
            break;
        default:
            break;
        }

        SmartDashboard.putString("vision/rioStatus", this.connectionStatus);

    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void reset() {

    }

}
