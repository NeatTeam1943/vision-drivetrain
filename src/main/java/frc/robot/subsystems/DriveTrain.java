// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX m_leftFront;
  private WPI_TalonFX m_leftRear;
  private WPI_TalonFX m_rightFront;
  private WPI_TalonFX m_rightRear;

  private MotorControllerGroup m_leftMotors;
  private MotorControllerGroup m_rightMotors;

  private DifferentialDrive m_drive;

  /**
   * 
   * Constructs a new instance of the DriveTrain class.
   * 
   * Initializes and configures the TalonFX motor controllers and sets up the
   * differential drive.
   */
  public DriveTrain() {
    m_leftFront = new WPI_TalonFX(DriveTrainConstants.kLeftFrontPort);
    m_leftRear = new WPI_TalonFX(DriveTrainConstants.kLeftRearPort);
    m_rightFront = new WPI_TalonFX(DriveTrainConstants.kRightFrontPort);
    m_rightRear = new WPI_TalonFX(DriveTrainConstants.kRightRearPort);

    m_leftFront.setNeutralMode(NeutralMode.Brake);
    m_leftRear.setNeutralMode(NeutralMode.Brake);
    m_rightFront.setNeutralMode(NeutralMode.Brake);
    m_rightRear.setNeutralMode(NeutralMode.Brake);

    m_rightFront.setInverted(true);
    m_rightRear.setInverted(true);

    m_leftMotors = new MotorControllerGroup(m_leftRear, m_leftFront);
    m_rightMotors = new MotorControllerGroup(m_rightRear, m_rightFront);

    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  }

  /**
   * 
   * Drives the robot in arcade mode using the provided joystick.
   * 
   * @apiNote ----Currently redundent----
   * 
   * @param joystick The XboxController instance used to control the robot's
   *                 movement.
   */
  public void driveArcade(XboxController joystick) {
    double mov = -joystick.getLeftY();
    double rot = joystick.getRightX();

    m_drive.arcadeDrive(mov, rot, true);
  }

  /**
   * 
   * Retrieves the DifferentialDrive instance used for driving the robot.
   * 
   * @return The DifferentialDrive object representing the robot's drive system.
   */
  public DifferentialDrive getDrive() {
    return m_drive;
  }

  public void setSpeed(double speed) {
    m_leftFront.set(speed);
    m_leftRear.set(speed);
    m_rightFront.set(speed);
    m_rightRear.set(speed);
  }

  public void setVoltages(double volt) {
    m_leftFront.setVoltage(volt);
    m_leftRear.setVoltage(volt);
    m_rightFront.setVoltage(volt);
    m_rightRear.setVoltage(volt);
  }

  public double getDistance() {
    double leftAvgPos = (m_leftFront.getSelectedSensorPosition() + m_leftRear.getSelectedSensorPosition()) / 2;
    double rightAvgPos = (m_rightFront.getSelectedSensorPosition() + m_rightRear.getSelectedSensorPosition()) / 2;
    double centralAvg = (leftAvgPos + rightAvgPos) / 4;
    double centralMotorAvg = centralAvg / DriveTrainConstants.kEncoderResolution;
    double centralWheelAvg = centralMotorAvg / DriveTrainConstants.kMotorToWheelRatio;

    return (-centralWheelAvg * DriveTrainConstants.kWheelCircumefrence) / 100;
  }

  public void resetEncoders() {
    m_leftFront.setSelectedSensorPosition(0);
    m_leftRear.setSelectedSensorPosition(0);
    m_rightFront.setSelectedSensorPosition(0);
    m_rightRear.setSelectedSensorPosition(0);
  }
}
