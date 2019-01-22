/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.lib.VersionData;
import frc.lib.loops.Looper;
import frc.robot.subsystems.Drive;

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
  private SubsystemManager Manager = new SubsystemManager(Arrays.asList(Drive.getInstance()));
  private Looper EnabledLoops = new Looper();
  private Looper DisabledLoops = new Looper();
  @Override
  public void robotInit() {
      VersionData.doVersionID();
      Manager.registerEnabledLoops(EnabledLoops);
      Manager.registerDisabledLoops(DisabledLoops);
  }

  @Override
  public void autonomousInit() {
    EnabledLoops.start();
    DisabledLoops.stop();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    EnabledLoops.start();
    DisabledLoops.stop();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledInit() {
    EnabledLoops.stop();
    DisabledLoops.start();
  }
}
