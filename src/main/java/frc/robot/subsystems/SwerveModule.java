package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private DutyCycleEncoder magEncoder;
  private RelativeEncoder driveEncoder;
  private RelativeEncoder angleEncoder;

  private PIDController angleController;
    
  public SwerveModule(CANSparkMax angleMotor, CANSparkMax driveMotor, DutyCycleEncoder magEncoder, double offset, boolean driveMotorReversed, boolean angleMotorReversed) {
    angleMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setIdleMode(IdleMode.kBrake);
    this.angleMotor = angleMotor;
    this.driveMotor = driveMotor;
    this.angleMotor.setInverted(angleMotorReversed);
    this.driveMotor.setInverted(driveMotorReversed);

    this.magEncoder = magEncoder;
    driveEncoder = driveMotor.getEncoder();
    angleEncoder = angleMotor.getEncoder();
    driveEncoder.setPositionConversionFactor(KDriveMotorRotToMeter);
    driveEncoder.setVelocityConversionFactor(KDriveMotorRPMToMetersPerSec);
    angleEncoder.setPositionConversionFactor(KAngleMotorRotToDeg);

    angleController = new PIDController(KAngleP, KAngleI, KAngleD);
    angleController.enableContinuousInput(-180, 180); // Tells PIDController that 180 deg is same in both directions

    setAbsoluteOffset(offset);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // If no controller input, set angle and drive motor to 0
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      angleMotor.set(0);
      driveMotor.set(0);
      return;
    }

    double angleMotorOutput;
    double driveMotorOutput;

    Rotation2d currentAngleR2D = getAngleR2D();
    desiredState = SwerveModuleState.optimize(desiredState, currentAngleR2D);

    // Angle calculation
    angleMotorOutput = angleController.calculate(getAngleDeg(), -desiredState.angle.getDegrees()); // TRYING NEGATIVE FOR ROTATION
    // angleMotorOutput = angleController.calculate(getAngleDeg(), -desiredState.angle.getDegrees());
    angleMotor.set(angleMotorOutput);

    // Drive calculation
    driveMotorOutput = desiredState.speedMetersPerSecond / KPhysicalMaxDriveSpeedMPS;

    driveMotor.set(driveMotorOutput);
  }

  public void stop() {
    driveMotor.set(0);
    angleMotor.set(0);
  }

  public void resetRelEncoders() {
    driveEncoder.setPosition(0);
    angleEncoder.setPosition(getMagDeg());
  }

  public void setAbsoluteOffset(double offset) {
    magEncoder.setPositionOffset(offset);
  }
  
  public double getAbsoluteOffset() {
    return magEncoder.getPositionOffset();
  }

  // Drive Encoder getters
  public double getDriveEncoderPos() {
    return driveEncoder.getPosition();
  }

  public double getDriveEncoderVel() {
    return driveEncoder.getVelocity();
  }

  // Angle Encoder getters
  public double getMagRotations() {
    double pos = magEncoder.get() % 1;
    return pos;
  }

  public double getMagDeg() {
    return -getMagRotations() * 360 % 360;
  }

  public double getAngleDeg() {
    return angleEncoder.getPosition() % 360;
  }

  public Rotation2d getAngleR2D() {
    return Rotation2d.fromDegrees(getAngleDeg()); 
  }
}
