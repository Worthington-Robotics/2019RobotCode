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
import frc.robot.subsystems.BlinkyLights;

import java.util.Arrays;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  SubsystemManager manager = new SubsystemManager(Arrays.asList(BlinkyLights.getInstance()));
  Looper enabled = new Looper();
  Looper disabled = new Looper();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
      VersionData.doVersionID();
      manager.registerEnabledLoops(enabled);
      manager.registerDisabledLoops(disabled);

  }

  public void disabledInit(){
    enabled.stop();
    disabled.start();
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    enabled.start();
    disabled.stop();
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

}
