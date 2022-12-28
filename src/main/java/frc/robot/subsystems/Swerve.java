package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import lib.RunEndCommand;

import static frc.robot.Constants.SwerveConstants.*;

public class Swerve extends SubsystemBase {
  //create swerve modules
  private final SwerveModule[] swerveModules = {
        new SwerveModule(
              kDriveMotorId[FRONT_LEFT],
              kSpinningMotorId[FRONT_LEFT],
              kDriveMotorReversed[FRONT_LEFT],
              kSpinningMotorReversed[FRONT_LEFT],
              kAbsEncoderChannel[FRONT_LEFT],
              kOffsetAngle[FRONT_LEFT]),
        new SwerveModule(
              kDriveMotorId[FRONT_RIGHT],
              kSpinningMotorId[FRONT_RIGHT],
              kDriveMotorReversed[FRONT_RIGHT],
              kSpinningMotorReversed[FRONT_RIGHT],
              kAbsEncoderChannel[FRONT_RIGHT],
              kOffsetAngle[FRONT_RIGHT]),
        new SwerveModule(
              kDriveMotorId[BACK_LEFT],
              kSpinningMotorId[BACK_LEFT],
              kDriveMotorReversed[BACK_LEFT],
              kSpinningMotorReversed[BACK_LEFT],
              kAbsEncoderChannel[BACK_LEFT],
              kOffsetAngle[BACK_LEFT]),
        new SwerveModule(
              kDriveMotorId[BACK_RIGHT],
              kSpinningMotorId[BACK_RIGHT],
              kDriveMotorReversed[BACK_RIGHT],
              kSpinningMotorReversed[BACK_RIGHT],
              kAbsEncoderChannel[BACK_RIGHT],
              kOffsetAngle[BACK_RIGHT])};

  private final AHRS _gyro = new AHRS(SPI.Port.kMXP);

  public Swerve() {
    resetGyro();
  }

//  private final SwerveDriveOdometry _odometry = new SwerveDriveOdometry(
//        kSwerveKinematics,

//        getRotation2d());

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

    /*
    @Override
    public void periodic() {
        _odometry.update(
                getRotation2d(),
                swerveModules[FRONT_LEFT].getState(),
                swerveModules[FRONT_RIGHT].getState(),
                swerveModules[BACK_LEFT].getState(),
                swerveModules[BACK_RIGHT].getState());
        SmartDashboard.putNumber("Robot heading to: ", getDegrees());
        SmartDashboard.putString("Robot location: ", getOdomertyPose().getTranslation().toString());
    }
*/

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Swerve");
    builder.clearProperties();

    builder.addDoubleProperty("FL angle", swerveModules[FRONT_LEFT]::getAbsEncoderResetRad, null);
    builder.addDoubleProperty("FR angle", swerveModules[FRONT_RIGHT]::getAbsEncoderResetRad, null);
    builder.addDoubleProperty("BL angle", swerveModules[BACK_LEFT]::getAbsEncoderResetRad, null);
    builder.addDoubleProperty("BR angle", swerveModules[BACK_RIGHT]::getAbsEncoderResetRad, null);
  }

//  @Override
//  public void periodic() {
//    if (Timer.getFPGATimestamp() - time > 10 && swerveModules[FRONT_LEFT].getDriveVelocity() < 0.01){
//      swerveModules[FRONT_LEFT].resetSpinningEncoder();
//      swerveModules[FRONT_RIGHT].resetSpinningEncoder();
//      swerveModules[BACK_LEFT].resetSpinningEncoder();
//      swerveModules[BACK_RIGHT].resetSpinningEncoder();
//
//      time = Timer.getFPGATimestamp();
//    }
//  }

  public void stopModules() {
    swerveModules[0].stop();
    swerveModules[1].stop();
    swerveModules[2].stop();
    swerveModules[3].stop();
  }

  public void setModulesStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kPhysicalMaxSpeedMetersPerSecond);
    swerveModules[0].setDesiredState(states[0]);
    swerveModules[1].setDesiredState(states[1]);
    swerveModules[2].setDesiredState(states[2]);
    swerveModules[3].setDesiredState(states[3]);
  }

  public Command resetModulesCommand(){
    return new RunCommand(
          ()-> {
            swerveModules[0].spinTo(0);
            swerveModules[1].spinTo(0);
            swerveModules[2].spinTo(0);
            swerveModules[3].spinTo(0);
          },
          this)
          .until(
                swerveModules[0].isReset
                      .and(swerveModules[1].isReset)
                      .and(swerveModules[2].isReset)
                      .and(swerveModules[3].isReset))
          .andThen(new PrintCommand("swerve is reset"),
                new InstantCommand(this::stopModules), new InstantCommand(this::resetEncoders));
  }

  private static double deadBandFix(double speed) {
    double joystickDeadBand = 0.2;
    return Math.abs(speed) < joystickDeadBand ? 0 : speed;
  }

  public Command driveSwerveCommand(DoubleSupplier xSpeedSupplier,
                                    DoubleSupplier ySpeedSupplier,
                                    DoubleSupplier spinningSpeedSupplier,
                                    BooleanSupplier fieldOriented) {
    final SlewRateLimiter xLimiter = new SlewRateLimiter(kTeleDriveMaxAccelerationUnitsPerSecond),
          yLimiter = new SlewRateLimiter(kTeleDriveMaxAccelerationUnitsPerSecond),
          spinningLimiter = new SlewRateLimiter(kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    return new RunEndCommand(
          () -> {
            //create the speeds for x,y and spinning and using a deadBand and Limiter to fix edge cases
            double xSpeed = xLimiter.calculate(deadBandFix(xSpeedSupplier.getAsDouble())) * kTeleDriveMaxSpeedMetersPerSecond,
                  ySpeed = yLimiter.calculate(deadBandFix(ySpeedSupplier.getAsDouble())) * kTeleDriveMaxSpeedMetersPerSecond,
                  spinningSpeed = spinningLimiter.calculate(deadBandFix(spinningSpeedSupplier.getAsDouble())) * kTeleDriveMaxAngularSpeedRadiansPerSecond;

            // create a CassisSpeeds object and apply it the speeds
            ChassisSpeeds chassisSpeeds = fieldOriented.getAsBoolean() ?
                  ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, spinningSpeed, getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, spinningSpeed);

            //use the ChassisSpeedsObject to create an array of SwerveModuleStates
            SwerveModuleState moduleStates[] = kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);

            //apply the array to the swerve modules of the robot
            setModulesStates(moduleStates);
          },
          this::stopModules, this);
  }

  public void resetEncoders(){
      swerveModules[0].resetEncoders();
      swerveModules[1].resetEncoders();
      swerveModules[2].resetEncoders();
      swerveModules[3].resetEncoders();
  }

  public Command resetGyroCommand(){
    return new InstantCommand(this::resetGyro);
  }

//  public Command resetOdometryCommand(Pose2d pose) {
//    return new InstantCommand(() -> _odometry.resetPosition(pose, getRotation2d()));
//  }
}
