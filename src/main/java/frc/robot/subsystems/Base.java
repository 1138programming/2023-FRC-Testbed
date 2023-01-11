package frc.robot.subsystems;

import static frc.robot.Constants.*;

import java.io.File;
import java.io.IOException;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Base extends SubsystemBase {

  private SwerveModule frontLeftModule;
  private SwerveModule frontRightModule;
  private SwerveModule backLeftModule;
  private SwerveModule backRightModule;

  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  public static AHRS gyro;

  private Pose2d pose;

  public Base() {
    frontLeftModule = new SwerveModule(
      new CANSparkMax(KFrontLeftAngleID, MotorType.kBrushless),
      new CANSparkMax(KFrontLeftDriveID, MotorType.kBrushless),
      new DutyCycleEncoder(KFrontLeftMagEncoderID),
      Rotation2d.fromDegrees(KFrontLeftOffset)
    );
    frontRightModule = new SwerveModule(
      new CANSparkMax(KFrontRightAngleID, MotorType.kBrushless), 
      new CANSparkMax(KFrontRightDriveID, MotorType.kBrushless), 
      new DutyCycleEncoder(KFrontRightMagEncoderID), 
      Rotation2d.fromDegrees(KFrontRightOffset)
    );
    backLeftModule = new SwerveModule(
      new CANSparkMax(KBackLeftAngleID, MotorType.kBrushless), 
      new CANSparkMax(KBackLeftDriveID, MotorType.kBrushless), 
      new DutyCycleEncoder(KBackLeftMagEncoderID), 
      Rotation2d.fromDegrees(KBackLeftOffset)
    );
    backRightModule = new SwerveModule(
      new CANSparkMax(KBackRightAngleID, MotorType.kBrushless), 
      new CANSparkMax(KBackRightDriveID, MotorType.kBrushless), 
      new DutyCycleEncoder(KBackRightMagEncoderID), 
      Rotation2d.fromDegrees(KBackRightOffset)
    );

    kinematics = new SwerveDriveKinematics(
      KFrontLeftLocation, KFrontRightLocation,
      KBackLeftLocation, KBackRightLocation
    );
    odometry = new SwerveDriveOdometry(kinematics, getHeading(), getPositions());
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();
  }

  public SwerveModulePosition[] getPositions() {
    // SwerveModuleState[] states = new SwerveModuleState[4];

    // states[0] = new SwerveModuleState(frontLeftModule.getDriveEncoderVel(), frontLeftModule.getAngleR2D());
    // states[1] = new SwerveModuleState(frontRightModule.getDriveEncoderVel(), frontRightModule.getAngleR2D());
    // states[2] = new SwerveModuleState(backLeftModule.getDriveEncoderVel(), backLeftModule.getAngleR2D());
    // states[3] = new SwerveModuleState(backRightModule.getDriveEncoderVel(), backRightModule.getAngleR2D());
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    positions[0] = new SwerveModulePosition(frontLeftModule.getDriveEncoderPos(), frontLeftModule.getAngleR2D());
    positions[1] = new SwerveModulePosition(frontRightModule.getDriveEncoderPos(), frontRightModule.getAngleR2D());
    positions[2] = new SwerveModulePosition(backLeftModule.getDriveEncoderPos(), backLeftModule.getAngleR2D());
    positions[3] = new SwerveModulePosition(backRightModule.getDriveEncoderPos(), backRightModule.getAngleR2D());

    return positions;
  }

  @Override
  public void periodic() {
    pose = odometry.update(getHeading(), getPositions());
  }

  //Getters
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  class SwerveModule {
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor; 

    private DutyCycleEncoder magEncoder;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder angleEncoder;

    private PIDController driveController;
    private PIDController angleController;

    private Rotation2d offset;

    SwerveModule(CANSparkMax angleMotor, CANSparkMax driveMotor, DutyCycleEncoder magEncoder, Rotation2d offset) {
      angleMotor.setIdleMode(IdleMode.kBrake);
      driveMotor.setIdleMode(IdleMode.kBrake);
      this.angleMotor = angleMotor;
      this.driveMotor = driveMotor;

      this.magEncoder = magEncoder;
      driveEncoder = driveMotor.getEncoder();
      angleEncoder = angleMotor.getEncoder();
      driveEncoder.setPositionConversionFactor(KDriveMotorRotToMeter);
      driveEncoder.setVelocityConversionFactor(KDriveMotorRPMToMetersPerSec);
      angleEncoder.setPositionConversionFactor(KAngleMotorRotToDeg);

      driveController = new PIDController(KDriveP, KDriveI, KDriveD);
      angleController = new PIDController(KAngleP, KAngleI, KAngleD);
      angleController.enableContinuousInput(-180, 180); // Tells PIDController that 180 deg is same in both directions

      // driveMotor.
      this.offset = offset;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
      double angleMotorOutput;
      double driveMotorOutput;

      // If no controller input, set angle and drive motor to 0
      if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
        angleMotor.set(0);
        driveMotor.set(0);
        return;
      }

      Rotation2d currentAngleR2D = getAngleR2D();
      desiredState = SwerveModuleState.optimize(desiredState, currentAngleR2D);

      // Angle calculation
      Rotation2d rotationDelta = desiredState.angle.minus(currentAngleR2D);
      double deltaDeg = rotationDelta.getDegrees();

      if (Math.abs(deltaDeg) < 2) {
        angleMotorOutput = 0;
      }
      else {
        angleMotorOutput = angleController.calculate(getAngleDeg(), desiredState.angle.getDegrees());
      }

      // Drive calculation
      driveMotorOutput = desiredState.speedMetersPerSecond / KPhysicalMaxDriveSpeedMPS;
    }

    public void setDriveGains() {
      driveController.setP(SmartDashboard.getNumber("drive kp", 0));
      driveController.setP(SmartDashboard.getNumber("drive ki", 0));
      driveController.setP(SmartDashboard.getNumber("drive kd", 0));
    }

    public void setAngleGains() {
      driveController.setP(SmartDashboard.getNumber("angle kp", 0));
      driveController.setP(SmartDashboard.getNumber("angle ki", 0));
      driveController.setP(SmartDashboard.getNumber("angle kd", 0));
    }

    public void resetRelEncoders() {
      driveEncoder.setPosition(0);
      angleEncoder.setPosition(getAngleDeg() - offset.getDegrees());
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
      return magEncoder.get();
    }
    public double getAngleDegRaw() {
      double angle = getMagRotations() * 360 % 360;
      return angle;
    }
    public double getAngleDeg() {
      return angleEncoder.getPosition() % 360;
    }
    public Rotation2d getAngleR2D() {
      return Rotation2d.fromDegrees(getAngleDeg()); 
    }
  }
}
