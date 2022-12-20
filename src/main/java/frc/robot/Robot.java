// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final DutyCycleEncoder encoderA = new DutyCycleEncoder(0);
  private final DutyCycleEncoder encoderB = new DutyCycleEncoder(1);
//  private final CANSparkMax angle = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax angleB = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax angleC = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax angleD = new CANSparkMax(17, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax drive = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax driveB = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax driveC = new CANSparkMax(16, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax driveD = new CANSparkMax(18, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final Joystick joystick = new Joystick(0);

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    encoderA.reset();
    encoderB.reset();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("posA", encoderA.getAbsolutePosition());
    SmartDashboard.putNumber("posB", encoderB.getAbsolutePosition());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    angleC.follow(angleB);
    angleD.follow(angleB);

    driveB.follow(drive);
    driveC.follow(drive);
    driveD.follow(drive);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drive.set(joystick.getY() < 0.15 && joystick.getY() > -0.15? 0: -joystick.getY() / 1.5);
    angleB.set(joystick.getX() / 3);

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
