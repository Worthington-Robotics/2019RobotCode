/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.lib.statemachine.StateMachine;
import frc.lib.util.DriveSignal;
import frc.lib.util.VersionData;
import frc.lib.loops.Looper;
import frc.robot.autoactiongroups.GoTenFeet;
import frc.robot.subsystems.*;

import java.util.Arrays;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    /**
    * This function is run when the robot is first started up and should be used
    * for any initialization code.
    */
    private SubsystemManager Manager = new SubsystemManager(Arrays.asList(
          Drive.getInstance(),
          PoseEstimator.getInstance(),
          Manipulator.getInstance(),
          Arm.getInstance(),
          Logger.getInstance()
    ));
    private Looper EnabledLoops = new Looper();
    private Looper DisabledLoops = new Looper();
    private OI Oi = new OI();

    public void robotPeriodic(){
    Manager.outputTelemetry();
    }

    @Override
    public void robotInit() {
        VersionData.doVersionID();
        Manager.registerEnabledLoops(EnabledLoops);
        Manager.registerDisabledLoops(DisabledLoops);
        Logger.getInstance().addNumberKeys(Constants.NUMBER_KEYS);
    }

    @Override
    public void autonomousInit() {
        PoseEstimator.getInstance().reset();
        Drive.getInstance().reset();
        EnabledLoops.start();
        DisabledLoops.stop();
        StateMachine.runMachine(new GoTenFeet());
    }

    @Override
    public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        PoseEstimator.getInstance().reset();
        Drive.getInstance().reset();
        EnabledLoops.start();
        DisabledLoops.stop();
        Drive.getInstance().setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void teleopPeriodic() {
    Scheduler.getInstance().run();
    }

    @Override
    public void testInit() {
        EnabledLoops.start();
        DisabledLoops.stop();
        Drive.getInstance().reset();
    }

    @Override
    public void testPeriodic() {
    Scheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        EnabledLoops.stop();
        DisabledLoops.start();
    }
}
