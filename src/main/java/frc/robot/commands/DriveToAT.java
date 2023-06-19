// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveTrain;

public class DriveToAT extends CommandBase {
  private XboxController m_joystick;
  private PIDController m_controller;

  private DriveTrain m_drive;
  private PhotonCamera m_camera;

  /**
   * 
   * Constructs a new instance of the DriveToAT class.
   * 
   * @param drive    The DriveTrain instance used for controlling the robot's
   *                 drive.
   * 
   * @param camera   The PhotonCamera instance used for vision processing.
   * 
   * @param joystick The XboxController instance used for controlling the robot's
   *                 movement.
   */
  public DriveToAT(DriveTrain drive, PhotonCamera camera, XboxController joystick) {
    m_drive = drive;
    m_controller = new PIDController(PIDConstants.kp, PIDConstants.ki, PIDConstants.kd);

    m_joystick = joystick;
    m_camera = camera;

    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    var result = m_camera.getLatestResult();

    double forwardSpeed = 0;
    double rotationSpeed = m_joystick.getLeftX();

    if (m_joystick.getAButton()) {
      if (result.hasTargets()) {
        double range = PhotonUtils.calculateDistanceToTargetMeters(
            VisionConstants.kCameraHeightMeters,
            VisionConstants.kTargetHeightMeters,
            VisionConstants.kCameraPitchRadians,
            Units.degreesToRadians(result.getBestTarget().getPitch()));

        forwardSpeed = -m_controller.calculate(range, VisionConstants.kGoalRangeMeters);
      } else {
        forwardSpeed = 0;
      }
    } else {
      forwardSpeed = -m_joystick.getRightY();
    }

    m_drive.getDrive().arcadeDrive(forwardSpeed, rotationSpeed);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
