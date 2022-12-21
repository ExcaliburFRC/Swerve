// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class SwerveConstants {

    public static final double kTrackWidthMeters = 0.5842;
    public static final double kWheelBaseMeters = 0.5842;
    public static final SwerveDriveKinematics kSwerveKinematics =
          new SwerveDriveKinematics(
                new Translation2d(kWheelBaseMeters/2,-kTrackWidthMeters/2),
                new Translation2d(kWheelBaseMeters/2,kTrackWidthMeters/2),
                new Translation2d(-kWheelBaseMeters/2,-kTrackWidthMeters/2),
                new Translation2d(-kWheelBaseMeters/2,kTrackWidthMeters/2));
    public static final double kPhysicalMaxSpeedMeterPerSec = 3;public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    public static final int kFrontLeftDriveMotorId = 18;
    public static final int kFrontRightDriveMotorId = 12;
    public static final int kBackLeftDriveMotorId = 16;
    public static final int kBackRightDriveMotorId = 14;

    public static final int kFrontLeftSpinningMotorId = 17;
    public static final int kFrontRightSpinningMotorId = 11;
    public static final int kBackLeftSpinningMotorId = 15;
    public static final int kBackRightSpinningMotorId = 13;

    public static final boolean kFrontLeftDriveEncoderReverse = false;
    public static final boolean kFrontRightDriveEncoderReverse = false;
    public static final boolean kBackLeftDriveEncoderReverse = true;
    public static final boolean kBackRightDriveEncoderReverse = false;

    public static final boolean kFrontLeftSpinningEncoderReverse = false;
    public static final boolean kFrontRightSpinningEncoderReverse = false;
    public static final boolean kBackLeftSpinningEncoderReverse = false;
    public static final boolean kBackRightSpinningEncoderReverse = false;

    public static final int kFrontLeftAbsEncoderChannel = 1;
    public static final int kFrontRightAbsEncoderChannel = 3;
    public static final int kBackLeftAbsEncoderChannel = 0;
    public static final int kBackRightAbsEncoderChannel = 2;

    public static final double kFrontLeftOffsetAngle = 0.063381251584531;
    public static final double kFrontRightOffsetAngle = 0.543378463584462;
    public static final double kBackRightOffsetAngle = 0.596312214907805;
    public static final double kBackLeftOffsetAngle = 0.653265316331633;

    public static final TrajectoryConfig kConfig = new TrajectoryConfig(
          kTeleDriveMaxSpeedMetersPerSecond,
          kMaxAccelerationMetersPerSecondSquared).setKinematics(kSwerveKinematics);
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
          new TrapezoidProfile.Constraints(
                kTeleDriveMaxAngularSpeedRadiansPerSecond,
                kMaxAngularAccelerationRadiansPerSecondSquared);
    public static final double kPXAuto = 0;
    public static final double kPYAuto = 0;
    public static final double kThetaAuto = 0;

    public static final PIDController xAutoController = new PIDController(0,0,0);//only kp needed
    public static final PIDController yAutoController = new PIDController(0,0,0);//only kp needed
    public static ProfiledPIDController thetaAutoController = new ProfiledPIDController(
            0,0,0,kThetaControllerConstraints);//only kp needed

  }

  public static final class ModuleConstants {
    //TODO update values
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); //127.0 / 1250.0;
    public static final double kDriveMotorGearRatio = 1 / 8.14;
    public static final double kDriveEncoderRot2Meters = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meters / 60;

    public static final double kTurningMotorGearRatio = 1 / 21.4285714;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
  }

  // driveCurrentLimit = 42.1
  // spinCurrentLimit = default
}
