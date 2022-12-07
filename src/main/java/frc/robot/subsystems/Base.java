package frc.robot.subsystems;

import static frc.robot.Constants.BaseConstants;
import static frc.robot.Constants.SwerveModuleConstants;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
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
                new CANSparkMax(BaseConstants.KFrontLeftAngleID, MotorType.kBrushless),
                new CANSparkMax(BaseConstants.KFrontLeftDriveID, MotorType.kBrushless),
                new DutyCycleEncoder(BaseConstants.KFrontLeftMagEncoderID),
                Rotation2d.fromDegrees(BaseConstants.KFrontLeftOffset)
            ),
            new SwerveModule(
                new CANSparkMax(BaseConstants.KFrontRightAngleID, MotorType.kBrushless), 
                new CANSparkMax(BaseConstants.KFrontRightDriveID, MotorType.kBrushless), 
                new DutyCycleEncoder(BaseConstants.KFrontRightMagEncoderID), 
                Rotation2d.fromDegrees(BaseConstants.KFrontRightOffset)
            ),
            new SwerveModule(
                new CANSparkMax(BaseConstants.KBackLeftAngleID, MotorType.kBrushless), 
                new CANSparkMax(BaseConstants.KBackLeftDriveID, MotorType.kBrushless), 
                new DutyCycleEncoder(BaseConstants.KBackLeftMagEncoderID), 
                Rotation2d.fromDegrees(BaseConstants.KBackLeftOffset)
            ),
            new SwerveModule(
                new CANSparkMax(BaseConstants.KBackRightAngleID, MotorType.kBrushless), 
                new CANSparkMax(BaseConstants.KBackRightDriveID, MotorType.kBrushless), 
                new DutyCycleEncoder(BaseConstants.KBackRightMagEncoderID), 
                Rotation2d.fromDegrees(BaseConstants.KBackRightOffset)
            )
        };

        kinematics = new SwerveDriveKinematics(
            BaseConstants.KFrontLeftLocation, BaseConstants.KFrontRightLocation,
            BaseConstants.KBackLeftLocation, BaseConstants.KBackRightLocation
        );
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d());
        gyro = new AHRS(SPI.Port.kMXP);

        gyro.reset();
    }

    @Override
    public void periodic() {
        // pose = odometry.update();
    }

    class SwerveModule {
        private double KAngleP;
        private double KAngleI;
        private double KAngleD;

        private double KDriveP;
        private double KDriveI;
        private double KDriveD;

        private CANSparkMax angleMotor;
        private CANSparkMax driveMotor; 

        private DutyCycleEncoder absoluteEncoder;
        private RelativeEncoder driveEncoder;
        private RelativeEncoder angleEncoder;

        private Rotation2d offset;

        SwerveModule(CANSparkMax angleMotor, CANSparkMax driveMotor, DutyCycleEncoder absoluteEncoder, Rotation2d offset) {
            KAngleP = SwerveModuleConstants.KAngleP;
            KAngleI = SwerveModuleConstants.KAngleI;
            KAngleD = SwerveModuleConstants.KAngleD;

            KDriveP = SwerveModuleConstants.KDriveP;
            KDriveI = SwerveModuleConstants.KDriveI;
            KDriveD = SwerveModuleConstants.KDriveD;

            this.angleMotor = angleMotor;
            this.driveMotor = driveMotor;

            this.absoluteEncoder = absoluteEncoder;
            this.offset = offset;
        }
    }
}
