// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DrivePid;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {
  private final DriveTrain m_drive;  

  private final PhotonCamera m_camera;

  private final XboxController m_joystick;

  private final DrivePid m_pidDrive; 

  public RobotContainer() {
    m_drive = new DriveTrain();

    m_camera = new PhotonCamera(VisionConstants.kCameraName);

    m_joystick = new XboxController(0);

    m_pidDrive = new DrivePid(m_drive);

    m_drive.resetEncoders();
    
    m_drive.setDefaultCommand(Commands.run(() -> m_drive.driveArcade(m_joystick), m_drive));

  }

  public Command getAutonomousCommand() {
    return m_pidDrive; 
  }
}
