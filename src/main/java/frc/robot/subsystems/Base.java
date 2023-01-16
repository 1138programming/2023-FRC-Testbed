package frc.robot.subsystems;

import static frc.robot.Constants.*;
import frc.robot.subsystems.SwerveModule;

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
  
  private double driveSpeedFactor;
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
      KFrontLeftOffset,
      KFrontLeftDriveReversed,
      KFrontLeftAngleReversed
    );
    frontRightModule = new SwerveModule(
      new CANSparkMax(KFrontRightAngleID, MotorType.kBrushless), 
      new CANSparkMax(KFrontRightDriveID, MotorType.kBrushless), 
      new DutyCycleEncoder(KFrontRightMagEncoderID), 
      KFrontRightOffset,
      KFrontRightDriveReversed,
      KFrontRightAngleReversed
    );
    backLeftModule = new SwerveModule(
      new CANSparkMax(KBackLeftAngleID, MotorType.kBrushless), 
      new CANSparkMax(KBackLeftDriveID, MotorType.kBrushless), 
      new DutyCycleEncoder(KBackLeftMagEncoderID), 
      KBackLeftOffset,
      KBackLeftDriveReversed,
      KBackLeftAngleReversed
    );
    backRightModule = new SwerveModule(
      new CANSparkMax(KBackRightAngleID, MotorType.kBrushless), 
      new CANSparkMax(KBackRightDriveID, MotorType.kBrushless), 
      new DutyCycleEncoder(KBackRightMagEncoderID), 
      KBackRightOffset,
      KBackRightDriveReversed,
      KBackRightAngleReversed
    );

    kinematics = new SwerveDriveKinematics(
      KFrontLeftLocation, KFrontRightLocation,
      KBackLeftLocation, KBackRightLocation
    );
    odometry = new SwerveDriveOdometry(kinematics, getHeading(), getPositions());
    driveSpeedFactor = KBaseDriveLowPercent;
    generateOdometryLog = false;
    startTime = RobotController.getFPGATime();
    odometryData = new ArrayList<String>();
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
    frontLeftModule.setDesiredState(states[2]);
    frontRightModule.setDesiredState(states[3]);
    backLeftModule.setDesiredState(states[0]);
    backRightModule.setDesiredState(states[1]);
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

  public void resetOdometry() {
    odometry.resetPosition(getHeading(), getPositions(), pose);
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

  public double getDriveSpeedFactor()
  {
    return driveSpeedFactor;
  }
  public void setDriveSpeedFactor(double set)
  {
    driveSpeedFactor = set;
  }
}
  
  