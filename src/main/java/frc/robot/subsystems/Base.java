package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Base {



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
