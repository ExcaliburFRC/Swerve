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

    public static final int FRONT_LEFT = 0;
    public static final int FRONT_RIGHT = 1;
    public static final int BACK_LEFT = 2;
    public static final int BACK_RIGHT = 3;

    public static final int[] kDriveMotorId = {18, 12, 16, 14};
    public static final int[] kSpinningMotorId = {17, 11, 15, 13};
    public static final boolean[] kDriveMotorReversed = {false, false, false, false};//TODO check
    public static final boolean[] kSpinningMotorReversed = {false, false, false, false};
    public static final int[] kAbsEncoderChannel = {1, 3, 0, 2};
    public static final double[] kOffsetAngle = {
          0.809,
          0.285,
          0.407,
          0.349
    };

    public static double kTolorance = 0.1;

    public static final double kTrackWidthMeters = 0.5842;
    public static final double kWheelBaseMeters = 0.5842;
    public static final SwerveDriveKinematics kSwerveKinematics =
          new SwerveDriveKinematics(
                new Translation2d(kWheelBaseMeters / 2, -kTrackWidthMeters / 2),
                new Translation2d(kWheelBaseMeters / 2, kTrackWidthMeters / 2),
                new Translation2d(-kWheelBaseMeters / 2, -kTrackWidthMeters / 2),
                new Translation2d(-kWheelBaseMeters / 2, kTrackWidthMeters / 2));

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;//TODO find
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;//TODO find
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;//TODO find

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1.5;//TODO choose
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 1.5;//TODO choose
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4; //TODO choose
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3; //TODO choose
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;//TODO choose

    // auto constants
    public static final TrajectoryConfig kConfig = new TrajectoryConfig(
          kTeleDriveMaxSpeedMetersPerSecond,
          kMaxAccelerationMetersPerSecondSquared).setKinematics(kSwerveKinematics);
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
          new TrapezoidProfile.Constraints(
                kTeleDriveMaxAngularSpeedRadiansPerSecond,
                kMaxAngularAccelerationRadiansPerSecondSquared);
    public static final double kPXAuto = 0;//TODO find
    public static final double kPYAuto = 0;//TODO find
    public static final double kThetaAuto = 0;//TODO find
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 8.14;
    public static final double kDriveEncoderRot2Meters = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meters / 60;
    public static final double kTurningMotorGearRatio = 1 / 21.4285714;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * Math.PI;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
  }
}
