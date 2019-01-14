package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Talon;
import frc.lib.loops.Loop;

//TYLER'S REALM
public class Arm extends Subsystem {

    public static final Arm m_Arm = new Arm();
    private TalonSRX armProx, armDist, armWrist;
    private double armProxPower, armDistPower, armWristPower;
    public static Arm getInstance() {return m_Arm;}

    //LOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP
    private final Loop mloop = new Loop() {
        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {

        }

        @Override
        public void onStop(double timestamp) {

        }
    };


    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {

    }

    @Override
    public void reset() {

    }
}

