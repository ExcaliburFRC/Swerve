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

import static frc.robot.Constants.SwerveConstants.*;
import static edu.wpi.first.math.MathUtil.applyDeadband;
import static frc.robot.Constants.SwerveConstants.Modules.*;

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

  public void resetGyro() {
    _gyro.reset();
  }

  private double getDegrees() {
    return Math.IEEEremainder(_gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getDegrees() + 90);
  }

  public Command resetModulesCommand(){
    return new FunctionalCommand(
            () -> {},
            ()-> {
            swerveModules[FRONT_LEFT].spinTo(0);
            swerveModules[FRONT_RIGHT].spinTo(0);
            swerveModules[BACK_LEFT].spinTo(0);
            swerveModules[BACK_RIGHT].spinTo(0);
            },
            (__)-> {
                stopModules();
                resetEncoders();
                new PrintCommand("modules reset").schedule();
            },
            ()-> swerveModules[FRONT_LEFT].isReset
                .and(swerveModules[FRONT_RIGHT].isReset)
                .and(swerveModules[BACK_LEFT].isReset)
                .and(swerveModules[BACK_RIGHT].isReset)
                .get(),
            this);
  }

  public Command driveSwerveCommand(
          DoubleSupplier xSpeedSupplier,
          DoubleSupplier ySpeedSupplier,
          DoubleSupplier spinningSpeedSupplier,
          BooleanSupplier fieldOriented) {

    final SlewRateLimiter
            xLimiter = new SlewRateLimiter(kMaxDriveAccelerationUnitsPerSecond),
            yLimiter = new SlewRateLimiter(kMaxDriveAccelerationUnitsPerSecond),
            spinningLimiter = new SlewRateLimiter(kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    return new FunctionalCommand(
            ()-> resetModulesCommand().schedule(false),
            () -> {
            //create the speeds for x,y and spinning and using a deadBand and Limiter to fix edge cases
            double xSpeed = xLimiter.calculate(applyDeadband(xSpeedSupplier.getAsDouble(), kDeadband)) * kMaxDriveSpeed,
                   ySpeed = yLimiter.calculate(applyDeadband(ySpeedSupplier.getAsDouble(), kDeadband)) * kMaxDriveSpeed,
                   spinningSpeed = spinningLimiter.calculate(applyDeadband(spinningSpeedSupplier.getAsDouble(), kDeadband)) * kMaxDriveTurningSpeed;

            // create a CassisSpeeds object and apply it the speeds
            ChassisSpeeds chassisSpeeds = fieldOriented.getAsBoolean() ?
                    ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, spinningSpeed, getRotation2d()) :
                    new ChassisSpeeds(xSpeed, ySpeed, spinningSpeed);

            //use the ChassisSpeedsObject to create an array of SwerveModuleStates
            SwerveModuleState moduleStates[] = kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);

            //apply the array to the swerve modules of the robot
            setModulesStates(moduleStates);
            },
            (__)-> {
                stopModules();
                resetEncoders();
            },
            ()-> false,
            this);
  }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve");
        builder.clearProperties();

        builder.addDoubleProperty("FL angle", swerveModules[FRONT_LEFT]::getResetRad, null);
        builder.addDoubleProperty("FR angle", swerveModules[FRONT_RIGHT]::getResetRad, null);
        builder.addDoubleProperty("BL angle", swerveModules[BACK_LEFT]::getResetRad, null);
        builder.addDoubleProperty("BR angle", swerveModules[BACK_RIGHT]::getResetRad, null);
    }

//  private double time = 0;
//  @Override
//  public void periodic() {
//    if (Timer.getFPGATimestamp() - time > 10 && swerveModules[SwerveModule.FRONT_LEFT].getDriveVelocity() < 0.01){
//      swerveModules[SwerveModule.FRONT_LEFT].resetSpinningEncoder();
//      swerveModules[FRONT_RIGHT].resetSpinningEncoder();
//      swerveModules[BACK_LEFT].resetSpinningEncoder();
//      swerveModules[BACK_RIGHT].resetSpinningEncoder();
//
//      time = Timer.getFPGATimestamp();
//    }
//  }

    private void setModulesStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, kPhysicalMaxSpeedMetersPerSecond);
        swerveModules[FRONT_LEFT].setDesiredState(states[FRONT_LEFT]);
        swerveModules[FRONT_RIGHT].setDesiredState(states[FRONT_RIGHT]);
        swerveModules[BACK_LEFT].setDesiredState(states[BACK_LEFT]);
        swerveModules[BACK_RIGHT].setDesiredState(states[BACK_RIGHT]);
    }

    public Command resetGyroCommand(){
        return new InstantCommand(this::resetGyro);
    }

    private void resetEncoders(){
        swerveModules[FRONT_LEFT].resetEncoders();
        swerveModules[FRONT_RIGHT].resetEncoders();
        swerveModules[BACK_LEFT].resetEncoders();
        swerveModules[BACK_RIGHT].resetEncoders();
    }

    private void stopModules() {
        swerveModules[FRONT_LEFT].stop();
        swerveModules[FRONT_RIGHT].stop();
        swerveModules[BACK_LEFT].stop();
        swerveModules[BACK_RIGHT].stop();
    }

    // autonomous code
//  private final SwerveDriveOdometry _odometry = new SwerveDriveOdometry(
//        kSwerveKinematics,
//
//        getRotation2d());
//
//  public Pose2d getOdomertyPose() {
//    return _odometry.getPoseMeters();

//  }
//    @Override
//    public void periodic() {
//        _odometry.update(
//                getRotation2d(),
//                swerveModules[SwerveModule.FRONT_LEFT].getState(),
//                swerveModules[FRONT_RIGHT].getState(),
//                swerveModules[BACK_LEFT].getState(),
//                swerveModules[BACK_RIGHT].getState());
//        SmartDashboard.putNumber("Robot heading to: ", getDegrees());
//        SmartDashboard.putString("Robot location: ", getOdomertyPose().getTranslation().toString());
//    }

//  public Command resetOdometryCommand(Pose2d pose) {
//    return new InstantCommand(() -> _odometry.resetPosition(pose, getRotation2d()));
//  }
}
