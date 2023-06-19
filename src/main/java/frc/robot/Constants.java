// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class VisionConstants {
        public static final double kCameraHeightMeters = Units.inchesToMeters(28.3464566929);
        public static final double kTargetHeightMeters = Units.inchesToMeters(23.6220472441);
        public static final double kCameraPitchRadians = Units.degreesToRadians(0);
        public static final double kGoalRangeMeters = Units.feetToMeters(3);
        public static final String kCameraName = "Microsoft_LifeCam_HD-3000";
    }

    public static class DriveTrainConstants {
        public static final int kLeftFrontPort = 1;
        public static final int kLeftRearPort = 2;
        public static final int kRightFrontPort = 3;
        public static final int kRightRearPort = 4;
    }

    public static class PIDConstants {
        public static final double kp = 0.1;
        public static final double ki = 0.0;
        public static final double kd = 0.0;
    }
}
