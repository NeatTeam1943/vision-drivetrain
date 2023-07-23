// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PidConstants;
import frc.robot.subsystems.DriveTrain;

public class DrivePid extends CommandBase {
  DriveTrain m_drive;
  PIDController m_controller;

  public DrivePid(DriveTrain drive) {
    m_drive = drive;
    m_controller = new PIDController(PidConstants.kp, PidConstants.ki, PidConstants.kd);

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double speed = m_controller.calculate(m_drive.getDistance(), 0.5);
    m_drive.setSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return m_controller.atSetpoint();
  }
}
