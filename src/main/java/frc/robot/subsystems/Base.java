package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Base extends SubsystemBase {

  private SwerveModule[] modules;

  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  public static AHRS gyro;

  private Pose2d pose;

  public Base() {
    modules = new SwerveModule[] {
      new SwerveModule(
        new CANSparkMax(KFrontLeftAngleID, MotorType.kBrushless),
        new CANSparkMax(KFrontLeftDriveID, MotorType.kBrushless),
        new DutyCycleEncoder(KFrontLeftMagEncoderID),
        Rotation2d.fromDegrees(KFrontLeftOffset)
      ),
      new SwerveModule(
        new CANSparkMax(KFrontRightAngleID, MotorType.kBrushless), 
        new CANSparkMax(KFrontRightDriveID, MotorType.kBrushless), 
        new DutyCycleEncoder(KFrontRightMagEncoderID), 
        Rotation2d.fromDegrees(KFrontRightOffset)
      ),
      new SwerveModule(
        new CANSparkMax(KBackLeftAngleID, MotorType.kBrushless), 
        new CANSparkMax(KBackLeftDriveID, MotorType.kBrushless), 
        new DutyCycleEncoder(KBackLeftMagEncoderID), 
        Rotation2d.fromDegrees(KBackLeftOffset)
      ),
      new SwerveModule(
        new CANSparkMax(KBackRightAngleID, MotorType.kBrushless), 
        new CANSparkMax(KBackRightDriveID, MotorType.kBrushless), 
        new DutyCycleEncoder(KBackRightMagEncoderID), 
        Rotation2d.fromDegrees(KBackRightOffset)
      )
    };

    kinematics = new SwerveDriveKinematics(
      KFrontLeftLocation, KFrontRightLocation,
      KBackLeftLocation, KBackRightLocation
    );
    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d());
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();
  }

  public SwerveModuleState[] getSpeeds() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    states[0] = new SwerveModuleState(-modules[0].getDriveEncoderVel(), modules[0].getAngleR2D());
    states[1] = new SwerveModuleState(-modules[1].getDriveEncoderVel(), modules[1].getAngleR2D());
    states[2] = new SwerveModuleState(-modules[2].getDriveEncoderVel(), modules[2].getAngleR2D());
    states[3] = new SwerveModuleState(modules[3].getDriveEncoderVel(), modules[3].getAngleR2D());

    return states;
  }

  @Override
  public void periodic() {
    pose = odometry.update(getHeading(), getSpeeds());
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    states[0] = new SwerveModuleState(modules[0].getDriveEncoderVel(), modules[0].getAngleR2D());
    states[1] = new SwerveModuleState(modules[1].getDriveEncoderVel(), modules[1].getAngleR2D());
    states[2] = new SwerveModuleState(modules[2].getDriveEncoderVel(), modules[2].getAngleR2D());
    states[3] = new SwerveModuleState(modules[3].getDriveEncoderVel(), modules[3].getAngleR2D());

    return states;
  }

  //Getters
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  class SwerveModule {
    private double angleP;
    private double angleI;
    private double angleD;

    private double driveP;
    private double driveI;
    private double driveD;

    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor; 

    private DutyCycleEncoder magEncoder;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder angleEncoder;

    private Rotation2d offset;

    SwerveModule(CANSparkMax angleMotor, CANSparkMax driveMotor, DutyCycleEncoder magEncoder, Rotation2d offset) {
      angleP = KAngleP;
      angleI = KAngleI;
      angleD = KAngleD;

      driveP = KDriveP;
      driveI = KDriveI;
      driveD = KDriveD;

      this.angleMotor = angleMotor;
      this.driveMotor = driveMotor;

      this.magEncoder = magEncoder;
      this.offset = offset;
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
      double angle = (getAngleDegRaw() + KDegPerRotation) % 360;
      return angle;
    }
    public Rotation2d getAngleR2D() {
      return Rotation2d.fromDegrees(getAngleDeg()); 
    }
  }
}
