// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private double m_power = 0;
  private TalonFX m_motor = new TalonFX(Constants.k_intakeMotor, "Default Name");
  private final double k_stallPower = .075;
  private Timer m_stallTimer = new Timer();
  public enum IntakeType {INTAKE, OUTTAKE, STOP}

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    setBrakeMode(false);
    m_stallTimer.reset();
    m_stallTimer.start();
  }

  public void setPower(double power) {
    Logger.log("IntakeSubsystem", 1, String.format("setPower: %f", power));
    m_power = power;
  }

  public void intake() {
    m_power = Constants.k_isCubeMode ? Constants.CubeConstants.k_intakePower : Constants.ConeConstants.k_intakePower;
  }

  public void outtake() {
    m_power = Constants.k_isCubeMode ? Constants.CubeConstants.k_outtakePower : Constants.ConeConstants.k_outtakePower;
  }

  public void stop() {
    m_power = Constants.k_isCubeMode ? 0 : k_stallPower;
  }

  public void yay(boolean intake) {
    System.out.println("hiiiii");
    m_motor.set(intake ? 1: -1);
  }

  public void setBrakeMode(boolean brake) {
    m_motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  public double getVelocity() {
    return m_motor.getVelocity().getValueAsDouble();
  }

  // Lower power if game piece is acquired
  public boolean isIntakeStalled() {
    double speed = getVelocity();
    boolean isIntakeStalled = Math.abs(speed) < 1 && Math.abs(m_power) > k_stallPower;
    if (isIntakeStalled) {
      if (m_stallTimer.get() > .25) {
        return true;
      }
      return false;
    } 
    m_stallTimer.reset();
    return false;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    isIntakeStalled();
    double power = k_stallPower + m_power;
    m_motor.set(.5);
    // Logger.log("IntakeSubsystem", 0, "periodic");
    // SmartDashboard.putNumber("Intake Speed", getVelocity());
  }
}
