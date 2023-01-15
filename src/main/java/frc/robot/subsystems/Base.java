package frc.robot.subsystems;

import static frc.robot.Constants.*;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
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
  private AHRS gyro;

  private Pose2d pose;
  
  private boolean generateOdometryLog;
  private long startTime;
  private ArrayList<String> odometryData;
  
  public Base() {
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();
    frontLeftModule = new SwerveModule(
      new CANSparkMax(KFrontLeftAngleID, MotorType.kBrushless),
      new CANSparkMax(KFrontLeftDriveID, MotorType.kBrushless),
      new DutyCycleEncoder(KFrontLeftMagEncoderID),
      Rotation2d.fromDegrees(KFrontLeftOffset),
      KFrontLeftInversion
    );
    frontRightModule = new SwerveModule(
      new CANSparkMax(KFrontRightAngleID, MotorType.kBrushless), 
      new CANSparkMax(KFrontRightDriveID, MotorType.kBrushless), 
      new DutyCycleEncoder(KFrontRightMagEncoderID), 
      Rotation2d.fromDegrees(KFrontRightOffset),
      KFrontRightInversion
    );
    backLeftModule = new SwerveModule(
      new CANSparkMax(KBackLeftAngleID, MotorType.kBrushless), 
      new CANSparkMax(KBackLeftDriveID, MotorType.kBrushless), 
      new DutyCycleEncoder(KBackLeftMagEncoderID), 
      Rotation2d.fromDegrees(KBackLeftOffset),
      KBackLeftInversion
    );
    backRightModule = new SwerveModule(
      new CANSparkMax(KBackRightAngleID, MotorType.kBrushless), 
      new CANSparkMax(KBackRightDriveID, MotorType.kBrushless), 
      new DutyCycleEncoder(KBackRightMagEncoderID), 
      Rotation2d.fromDegrees(KBackRightOffset),
      KBackRightInversion
    );

    kinematics = new SwerveDriveKinematics(
      KFrontLeftLocation, KFrontRightLocation,
      KBackLeftLocation, KBackRightLocation
    );
    odometry = new SwerveDriveOdometry(kinematics, getHeading(), getPositions());

    generateOdometryLog = false;
    startTime = RobotController.getFPGATime();
    odometryData = new ArrayList<String>();

    // backLeftModule.setAbsoluteOffset(.015);
  }

  // public void resetAbsolute() {
  //   frontLeftModule.resetAbsolute();
  //   frontRightModule.resetAbsolute();
  //   backLeftModule.resetAbsolute();
  //   backRightModule.resetAbsolute();
  // }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double maxDriveSpeedMPS) {
    xSpeed *= maxDriveSpeedMPS;
    ySpeed *= maxDriveSpeedMPS;
    rot *= KMaxAngularSpeed;
    
    //feeding parameter speeds into toSwerveModuleStates to get an array of SwerveModuleState objects
    SwerveModuleState[] states =
      kinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(gyro.getAngle()))
          // ? new ChassisSpeeds(xSpeed, ySpeed, rot)
          : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, KPhysicalMaxDriveSpeedMPS);

    //setting module states, aka moving the motors
    frontLeftModule.setDesiredState(states[0]);
    frontRightModule.setDesiredState(states[1]);
    backLeftModule.setDesiredState(states[2]);
    backRightModule.setDesiredState(states[3]);
}

  // recalibrates gyro offset
  public void resetGyro() {
    gyro.reset(); 
    gyro.setAngleAdjustment(0);
  }

  public void resetAllRelEncoders() {
    frontLeftModule.resetRelEncoders();
    frontRightModule.resetRelEncoders();
    backLeftModule.resetRelEncoders();
    backRightModule.resetRelEncoders();
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    positions[0] = new SwerveModulePosition(frontLeftModule.getDriveEncoderPos(), frontLeftModule.getAngleR2D());
    positions[1] = new SwerveModulePosition(frontRightModule.getDriveEncoderPos(), frontRightModule.getAngleR2D());
    positions[2] = new SwerveModulePosition(backLeftModule.getDriveEncoderPos(), backLeftModule.getAngleR2D());
    positions[3] = new SwerveModulePosition(backRightModule.getDriveEncoderPos(), backRightModule.getAngleR2D());

    return positions;
  }
  
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public double getHeadingDeg() {
    return gyro.getAngle();
  }

  public void setGenerateOdometryLog(boolean generateOdometryLog) {
    this.generateOdometryLog = generateOdometryLog;
  }
  public boolean getGenerateOdometryLog() {
    return generateOdometryLog;
  }

  private void genOdometryData() {
    long time =  RobotController.getFPGATime() - startTime;
    String s = ("" + (double) time / 1000000);
    s += "," + pose.getX() + "," + pose.getY() + "," + pose.getRotation().getDegrees();

    odometryData.add(s);
  }

  public void writeOdometryData() {
    try {
      FileWriter writer = new FileWriter("OdometryLog.txt");
      for (int i = 0; i < 1000; i++) {
        writer.write(odometryData.get(i));
      }
      writer.close();
    } catch (IOException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro", gyro.getAngle());

    SmartDashboard.putNumber("Front left module", frontLeftModule.getAngleDeg());
    SmartDashboard.putNumber("Front right module", frontRightModule.getAngleDeg());
    SmartDashboard.putNumber("Back left module", backLeftModule.getAngleDeg());
    SmartDashboard.putNumber("Back right module", backRightModule.getAngleDeg());

    SmartDashboard.putNumber("front left mag", frontLeftModule.getMagRotations());
    SmartDashboard.putNumber("front right mag", frontRightModule.getMagRotations());
    SmartDashboard.putNumber("back left mag", backLeftModule.getMagRotations());
    SmartDashboard.putNumber("back right mag", backRightModule.getMagRotations());

    SmartDashboard.putNumber("front left RAW", frontLeftModule.getAngleDegRaw());
    SmartDashboard.putNumber("front right RAW", frontRightModule.getAngleDegRaw());
    SmartDashboard.putNumber("back left RAW", backLeftModule.getAngleDegRaw());
    SmartDashboard.putNumber("back right RAW", backRightModule.getAngleDegRaw());

    SmartDashboard.putNumber("front left big", frontLeftModule.getAbsoluteOffset());
    SmartDashboard.putNumber("front right big", frontRightModule.getAbsoluteOffset());
    SmartDashboard.putNumber("back left big", backLeftModule.getAbsoluteOffset());
    SmartDashboard.putNumber("back right big", backRightModule.getAbsoluteOffset());

    SmartDashboard.putNumber("odometry X", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("odometry Y", odometry.getPoseMeters().getY());

    pose = odometry.update(getHeading(), getPositions());

    if (generateOdometryLog && odometryData.size() < 1000) {
      genOdometryData();
    }
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

    private boolean isInverted;
    
    SwerveModule(CANSparkMax angleMotor, CANSparkMax driveMotor, DutyCycleEncoder magEncoder, Rotation2d offset, boolean isInverted) {
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

      this.offset = offset;
      setAbsoluteOffset(offset.getDegrees());

      this.isInverted = isInverted;
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
      angleMotor.set(angleMotorOutput);

      // Drive calculation
      driveMotorOutput = desiredState.speedMetersPerSecond / KPhysicalMaxDriveSpeedMPS;

      if (isInverted) {
        driveMotorOutput *= -1;
      }

      driveMotor.set(driveMotorOutput);
    }

    public void setDriveGains() {
      driveController.setP(SmartDashboard.getNumber("drive kp", 0));
      driveController.setI(SmartDashboard.getNumber("drive ki", 0));
      driveController.setD(SmartDashboard.getNumber("drive kd", 0));
    }

    public void setAngleGains() {
      angleController.setP(SmartDashboard.getNumber("angle kp", 0));
      angleController.setI(SmartDashboard.getNumber("angle ki", 0));
      angleController.setD(SmartDashboard.getNumber("angle kd", 0));
    }

    public void resetRelEncoders() {
      driveEncoder.setPosition(0);
      // double angle = getAngleDegRaw();
      // if (angle < 0) {
      //   angle = 360 + angle;
      // }
      angleEncoder.setPosition(getMagRotations() * 360);
    }

    public void setAbsoluteOffset(double offset) {
      // magEncoder.setPositionOffset(offset);
    }
    public double getAbsoluteOffset() {
      return magEncoder.getPositionOffset();
    }

    // Drive Encoder getters
    public double getDriveEncoderPos() {
      return driveEncoder.getPosition();
    }
    // public void resetAbsolute() {
    //   magEncoder.reset();
    // }
    public double getDriveEncoderVel() {
      return driveEncoder.getVelocity();
    }

    // Angle Encoder getters
    public double getMagRotations() {
      double pos = magEncoder.get() % 1;
      if (pos < 0) {
        pos += 1;
      }
      pos = 1 - pos;
      return pos;
    }
    // public double getMagRotationsWithOffset() {
    //   double pos = getMagRotations() - magEncoder.getPositionOffset();
    //   if (pos < 0) {
    //     pos += 1;
    //   }
    //   return pos;
    // }
    public double getAngleDegRaw() {
      double angle = getMagRotations() * 360 % 360;
      // if (angle > 180) {
      //   angle -= 360;
      // } else if (angle < -180) {
      //   angle += 360;
      // }
      SmartDashboard.putNumber("BROOO", angle);
      return angle;
    }
    
    public double getAngleDeg() {
      return angleEncoder.getPosition() % 360;
      // SmartDashboard.putNumber("getName()", KAngleD)
      // return getMagRotations() * 360;
      // if (angle > 180) {
      //   angle = Math.abs(angle) - 360;
      // } /*else if (angle < -180) {
      //   angle += 360;
      // }*/
      // return angle;
    }

    public Rotation2d getAngleR2D() {
      return Rotation2d.fromDegrees(getAngleDeg()); 
    }
  }
}
