package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.RunEndCommand;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

public class Swerve extends SubsystemBase {
 
  //create swerve modules
  private final SwerveModule _frontLeft = new SwerveModule(
        Constants.SwerveConstants.kFrontLeftDriveMotorId,
        Constants.SwerveConstants.kFrontLeftSpinningMotorId,
        Constants.SwerveConstants.kFrontLeftDriveEncoderReverse,
        Constants.SwerveConstants.kFrontLeftSpinningEncoderReverse,
        Constants.SwerveConstants.kFrontLeftAbsEncoderChannel,
        Constants.SwerveConstants.kFrontLeftOffsetAngle);

  private final SwerveModule _frontRight = new SwerveModule(
        Constants.SwerveConstants.kFrontRightDriveMotorId,
        Constants.SwerveConstants.kFrontRightSpinningMotorId,
        Constants.SwerveConstants.kFrontRightDriveEncoderReverse,
        Constants.SwerveConstants.kFrontRightSpinningEncoderReverse,
        Constants.SwerveConstants.kFrontRightAbsEncoderChannel,
        Constants.SwerveConstants.kFrontRightOffsetAngle);

  private final SwerveModule _backLeft = new SwerveModule(
        Constants.SwerveConstants.kBackLeftDriveMotorId,
        Constants.SwerveConstants.kBackLeftSpinningMotorId,
        Constants.SwerveConstants.kBackLeftDriveEncoderReverse,
        Constants.SwerveConstants.kBackLeftSpinningEncoderReverse,
        Constants.SwerveConstants.kBackLeftAbsEncoderChannel,
        Constants.SwerveConstants.kBackLeftOffsetAngle);

  private final SwerveModule _backRight = new SwerveModule(
       Constants.SwerveConstants.kBackRightDriveMotorId,
        Constants.SwerveConstants.kBackRightSpinningMotorId,
        Constants.SwerveConstants.kBackRightDriveEncoderReverse,
        Constants.SwerveConstants.kBackRightSpinningEncoderReverse,
        Constants.SwerveConstants.kBackRightAbsEncoderChannel,
        Constants.SwerveConstants.kBackRightOffsetAngle);


 private final AHRS _gyro = new AHRS(SPI.Port.kMXP);
//  private final SwerveDriveOdometry _odometry = new SwerveDriveOdometry(
//        Constants.SwerveConstants.kSwerveKinematics,
//        getRotation2d());

  public Swerve() {
    resetGyro();
  }

//  public Pose2d getOdomertyPose() {
//    return _odometry.getPoseMeters();
//  }

  public void resetGyro() {
    _gyro.reset();
  }

  public double getDegrees() {
    return Math.IEEEremainder(_gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getDegrees());
  }
//
//  @Override
//  public void periodic() {
//    _odometry.update(
//          getRotation2d(),
//          _frontLeft.getState(),
//          _frontRight.getState(),
//          _backLeft.getState(),
//          _backRight.getState());
//    SmartDashboard.putNumber("Robot heading to: ", getDegrees());
//    SmartDashboard.putString("Robot location: ",getOdomertyPose().getTranslation().toString());
//  }


  @Override
  public void periodic() {
    SmartDashboard.putString("FrontLeft", _frontLeft.getState().toString());
    SmartDashboard.putString("FrontRight", _frontRight.getState().toString());
    SmartDashboard.putString("BackLeft", _backLeft.getState().toString());
    SmartDashboard.putString("BackRight", _backRight.getState().toString());
  }

  public void stopModules() {
    _frontLeft.stop();
    _frontRight.stop();
    _backLeft.stop();
    _backRight.stop();
  }

  public void setModulesStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states,
          Constants.SwerveConstants.kPhysicalMaxSpeedMeterPerSec);

    _frontLeft.setDesiredState(states[0]);
    _frontRight.setDesiredState(states[1]);
    _backLeft.setDesiredState(states[2]);
    _backRight.setDesiredState(states[3]);
  }
  private static double deadBandFix(double speed) {
    double joystickDeadBand = 0.2;
   return Math.abs(speed) < joystickDeadBand ? 0 : speed;
 }
  public Command driveSwerveCommand(DoubleSupplier xSpeedSupplier,
                                   DoubleSupplier ySpeedSupplier,
                                    DoubleSupplier spinningSpeedSupplier,
                                    BooleanSupplier fieldOriented) {
    final SlewRateLimiter xLimiter, yLimiter, spinningLimiter;

    xLimiter = new SlewRateLimiter(Constants.SwerveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    yLimiter = new SlewRateLimiter(Constants.SwerveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    spinningLimiter = new SlewRateLimiter(Constants.SwerveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    return new RunEndCommand(
          () -> {
            //create the speeds for x,y and spinning and using a deadBand and Limiter to fix edge cases
            double xSpeed = xLimiter.calculate(deadBandFix(xSpeedSupplier.getAsDouble()))
                  * Constants.SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond,
                  ySpeed = yLimiter.calculate(deadBandFix(ySpeedSupplier.getAsDouble()))
                        * Constants.SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond,
                 spinningSpeed = spinningLimiter.calculate(deadBandFix(spinningSpeedSupplier.getAsDouble()))
                        * Constants.SwerveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
            // create a CassisSpeeds object and apply it the speeds
            ChassisSpeeds chassisSpeeds = fieldOriented.getAsBoolean() ?
                  ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, spinningSpeed, getRotation2d()) :
                  new ChassisSpeeds(xSpeed, ySpeed, spinningSpeed);
            //use the ChassisSpeedsObject to create an array of SwerveModuleStates
            SwerveModuleState moduleStates[] =
                  Constants.SwerveConstants.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
            //apply the array to the swerve modules of the robot
            setModulesStates(moduleStates);
          },
          this::stopModules, this);
  }
//
//  public Command resetOdometryCommand(Pose2d pose) {
//    return new InstantCommand(() -> _odometry.resetPosition(pose, getRotation2d()));
//  }
}
