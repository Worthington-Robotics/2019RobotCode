package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Subsystem;

public class Vision extends Subsystem {

    public static final String NOT_INIT = "NOT_INIT";
    public static final String PING = "PING";
    public static final String PONG = "PONG";
    public static final String CONNECTED = "CONNECTED";
    public static final String RESET = "RESET";
    public static final String TABLE_NAME = "vision/connect";
    //private static double DIS = 0;

    // construct one and only 1 instance of this class
    private static Vision m_VisionInstance = new Vision();

    private String connectionStatus = NOT_INIT;
    //private Ultrasonic US1;

    private int timesConnected = 0;

    public static Vision getInstance() {
        return m_VisionInstance;
    }

    private Vision() {
        SmartDashboard.clearPersistent(TABLE_NAME);
        //US1 = new Ultrasonic(Constants.ULTRASONIC_IN_1, Constants.ULTRASONIC_OUT_1);
        //US1.update();
    }


    /*public double getDis()
    {
        return DIS;
    }*/
    @Override
    public synchronized void readPeriodicInputs() {

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
       /* DIS = US1.getDistance();
        if(DIS >= 120)
        {
            DIS = -1;
        }*/
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
                this.timesConnected++;
                break;
            default:
                break;
        }

    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Vision/rioStatus", this.timesConnected);
        //SmartDashboard.putNumber("Vision/Ultrasonic", getDis());// 15 for cargo // 34 for rocket
    }

    @Override
    public void reset() {

    }

}
