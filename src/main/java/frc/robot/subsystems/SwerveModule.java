package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import static java.lang.Math.PI;

public class SwerveModule {

  //create motors
  private final CANSparkMax _driveMotor;
  private final CANSparkMax _spinningMotor;

  private final RelativeEncoder _driveEncoder;
  private final RelativeEncoder _spinningEncoder;
  private final DutyCycleEncoder _absEncoder;
  private final double _absEncoderOffsetResetRad;
  private final double _absEncoderOffsetRad;
  private final int _absEncoderChannel;

  private final PIDController _spinningPIDController;

  public Trigger isReset = new Trigger(()-> Math.abs(getAbsEncoderResetRad()) < Constants.SwerveConstants.kTolorance).debounce(0.1);

  public SwerveModule(int driveMotorId,
                      int spinningMotorId,
                      boolean driveMotorReversed,
                      boolean spinningMotorReversed,
                      int absEncoderChannel,
                      double offsetAngle) {
    _absEncoderChannel = absEncoderChannel;
    _absEncoder = new DutyCycleEncoder(absEncoderChannel);
    _absEncoderOffsetResetRad = (offsetAngle * 2 - 1) * PI;
    _absEncoderOffsetRad = offsetAngle * 2 * PI;

    _driveMotor = new CANSparkMax(driveMotorId, CANSparkMax.MotorType.kBrushless);
    _spinningMotor = new CANSparkMax(spinningMotorId, CANSparkMax.MotorType.kBrushless);

    _driveMotor.setInverted(driveMotorReversed);
    _spinningMotor.setInverted(spinningMotorReversed);

    _driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    _spinningMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    _driveEncoder = _driveMotor.getEncoder();
    _spinningEncoder = _spinningMotor.getEncoder();

    _driveEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveEncoderRot2Meters);
    _driveEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    _spinningEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurningEncoderRot2Rad);
    _spinningEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kTurningEncoderRPM2RadPerSec);

    _spinningPIDController = new PIDController(Constants.ModuleConstants.kPTurning, 0, 0);
    _spinningPIDController.enableContinuousInput(-PI, PI);

    resetEncoders();
  }

  public double getDrivePosition() {
    return _driveEncoder.getPosition();
  }

  public double getSpinningPosition() {
    return _spinningEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return _driveEncoder.getVelocity();
  }

  public double getSpinningVelocity() {
    return _spinningEncoder.getVelocity();
  }

  // return the module angle between -PI to PI
  public double getAbsEncoderResetRad() {
    double angle = _absEncoder.getAbsolutePosition();
    angle = (angle * 2 - 1) * PI;
    angle -= _absEncoderOffsetResetRad;
    angle = angle < -PI ? 2 * PI + angle : angle;
    angle = angle > PI ? angle - (2 * PI) : angle;
    return angle;
  }

  // return the module angle between 0 to 2PI
  public double getAbsEncoderRad() {
    double angle = _absEncoder.getAbsolutePosition();
    angle = angle * 2 * Math.PI;
    angle -= _absEncoderOffsetRad;
    angle = angle < 0 ? 2 * Math.PI + angle : angle;
    return angle;
  }

  public void resetEncoders() {
    _driveEncoder.setPosition(0);
    _spinningEncoder.setPosition(getAbsEncoderRad());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsEncoderRad()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.01) {
      stop();
      return;
    }

//    state = SwerveModuleState.optimize(state, getState().angle);

    _driveMotor.set(state.speedMetersPerSecond / Constants.SwerveConstants.kPhysicalMaxSpeedMetersPerSecond);
    _spinningMotor.set(_spinningPIDController.calculate(getSpinningPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve [" + _absEncoderChannel + "] state ", state.toString());
  }

  public void spinTo(double setpoint){
    if (isReset.negate().get()) {
      _spinningMotor.set(Constants.ModuleConstants.kPTurning * (getAbsEncoderResetRad() - setpoint));
    }
    else {
      _spinningMotor.set(0);
    }
  }

  public void stop() {
    _driveMotor.set(0);
    _spinningMotor.set(0);
  }
}
