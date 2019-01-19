    package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.lib.loops.Loop;
import frc.robot.Constants;

//TYLER'S REALM
public class Arm extends Subsystem {

    public static final Arm m_Arm = new Arm();
    public TalonSRX armProx, armDist, armWrist;
    public double armProxPower, armDistPower, armWristPower;
    public static Arm getInstance() {return m_Arm;}
    public Arm() {
        armProx = new TalonSRX(Constants.ARM_PROXIMINAL);
        armDist = new TalonSRX(Constants.ARM_DISTAL);
        armWrist = new TalonSRX(Constants.ARM_WRIST);
    }
    //LOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP
    private final Loop aloop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            armProxPower = 0;
            armDistPower = 0;
            armWristPower = 0;
        }

        @Override
        public void onLoop(double timestamp) {

        }

        @Override
        public void onStop(double timestamp) {

        }
    };

    @Override
    public void readPeriodicInputs() {
        
    }

    @Override
    public void writePeriodicOutputs() {

    }
    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {

    }

    @Override
    public void reset() {
        armProxPower = 0;
        armDistPower = 0;
        armWristPower = 0;
    }
}
//I want to die, Cool Cool.
