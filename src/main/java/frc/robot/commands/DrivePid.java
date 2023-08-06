// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PidConstants;
import frc.robot.subsystems.DriveTrain;

public class DrivePid extends CommandBase {
  DriveTrain m_drive;
  PIDController m_controller;
  SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1);

  public DrivePid(DriveTrain drive) {
    m_drive = drive;
    m_controller = new PIDController(PidConstants.kp, PidConstants.ki, PidConstants.kd);

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_drive.resetEncoders();
  }

  @Override
  public void execute() {
    double speed = m_controller.calculate(m_drive.getDistance(), 2.0);
    double voltage = m_feedforward.calculate(1, 2);

    System.out.println("Speed: " + speed);
    System.out.println("Distance: " + m_drive.getDistance());
    m_drive.setVoltages(-speed + -voltage);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    System.out.println(m_controller.atSetpoint());
    return m_controller.atSetpoint();
  }
}
