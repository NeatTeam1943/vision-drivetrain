// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveTrain;

public class DriveToATNoPid extends CommandBase {
  private DriveTrain m_drive;
  private PhotonCamera m_camera;

  private boolean m_reachedTarget = false;

  public DriveToATNoPid(DriveTrain drive, PhotonCamera camera, XboxController joystick) {
    m_drive = drive;
    m_camera = camera;

    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    var result = m_camera.getLatestResult();

    if (result.hasTargets()) {
      double range = PhotonUtils.calculateDistanceToTargetMeters(
          VisionConstants.kCameraHeightMeters,
          VisionConstants.kTargetHeightMeters,
          VisionConstants.kCameraPitchRadians,
          Units.degreesToRadians(result.getBestTarget().getPitch()));

      System.out.println("range: " + range);

      m_reachedTarget = range > VisionConstants.kGoalRangeMeters;
    }

    m_drive.getDrive().arcadeDrive(DriveTrainConstants.kDriveSpeed, 0);
  }

  @Override
  public boolean isFinished() {
    return m_reachedTarget;
  }
}
