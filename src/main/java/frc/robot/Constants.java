// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class VisionConstants {
        public static final double kCameraHeightMeters = 0.73;
        public static final double kTargetHeightMeters = 1.57;
        public static final double kCameraPitchRadians = Units.degreesToRadians(-1.452);
        public static final String kCameraName = "Microsoft_LifeCam_HD-3000";

        // These values are subjected to change
        private static final double kFullDistance = 4.42;
        private static final double kGoal = 2.92;
        public static final double kGoalRangeMeters = kFullDistance - kGoal;
    }

    public static class DriveTrainConstants {
        public static final int kLeftFrontPort = 1;
        public static final int kLeftRearPort = 2;
        public static final int kRightFrontPort = 3;
        public static final int kRightRearPort = 4;
        public static final double kDriveSpeed = 0.11;

        public static final double k100msTo60sRatio = 600;
        public static final double kEncoderResolution = 2048;
        public static final double kMotorToWheelRatio = 4;
        public static final double kWheelCircumefrence = 47.879;
    }

    public static class PidConstants {
        public static final double kp = 0.0032492;
        public static final double ki = 0.0;
        public static final double kd = 0.0023599;
    }
}
