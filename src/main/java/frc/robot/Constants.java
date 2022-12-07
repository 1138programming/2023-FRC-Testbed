// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class SwerveModuleConstants {
        public static final double KAngleP = 0;
        public static final double KAngleI = 0;
        public static final double KAngleD = 0;
    
        public static final double KDriveP = 0;
        public static final double KDriveI = 0;
        public static final double KDriveD = 0;
    }
    
    public static class BaseConstants {
        public static final int KFrontLeftAngleID = 1;
        public static final int KFrontLeftDriveID = 2;

        public static final int KFrontRightAngleID = 3;
        public static final int KFrontRightDriveID = 4;

        public static final int KBackLeftAngleID = 5;
        public static final int KBackLeftDriveID = 6;

        public static final int KBackRightAngleID = 7;
        public static final int KBackRightDriveID = 8;

        
        public static final double KFrontLeftOffset = 0;
        public static final double KFrontRightOffset = 0;
        public static final double KBackLeftOffset = 0;
        public static final double KBackRightOffset = 0;

        public static final int KFrontLeftMagEncoderID = 1;
        public static final int KFrontRightMagEncoderID = 2;
        public static final int KBackLeftMagEncoderID = 3;
        public static final int KBackRightMagEncoderID = 4;

        public static final double KWheelDistanceFromCenter = 0.3683;

        public static final Translation2d KFrontLeftLocation = new Translation2d(
            KWheelDistanceFromCenter, KWheelDistanceFromCenter
        );
        public static final Translation2d KFrontRightLocation = new Translation2d(
            KWheelDistanceFromCenter, -KWheelDistanceFromCenter
        );
        public static final Translation2d KBackLeftLocation = new Translation2d(
            -KWheelDistanceFromCenter, KWheelDistanceFromCenter
        );
        public static final Translation2d KBackRightLocation = new Translation2d(
            -KWheelDistanceFromCenter, -KWheelDistanceFromCenter
        );        
    }
    
    public static final int KLeftBackDrive = 1;
}
