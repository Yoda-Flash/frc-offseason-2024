// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private CANSparkMax m_intakeMotor = new CANSparkMax(Constants.Intake.kIntakeMotorID, MotorType.kBrushless);

  /** Creates a new Intake. */
  public Intake() {

    //setInverted depending on which way the motor spins?

    m_intakeMotor.setSmartCurrentLimit(15, 15); //do later

    m_intakeMotor.burnFlash(); //does something to preven burnouts?
    
  }

  public void pickNote() {
    m_intakeMotor.set(Constants.Intake.kIntakeSpeed);
  }

  public InstantCommand intakeNote() {
    return new InstantCommand(this::pickNote, this);
  }

  public void dropNote() {
    m_intakeMotor.set(-Constants.Intake.kIntakeSpeed);
  }

  public InstantCommand releaseNote() {
    return new InstantCommand(this::dropNote, this);
  }

  public void zeroIntake() {
    m_intakeMotor.set(0);
  }

  public InstantCommand stopIntake() {
    return new InstantCommand(this::stopIntake, this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor Temperature", m_intakeMotor.getMotorTemperature());
    SmartDashboard.putNumber("Motor Current", m_intakeMotor.getOutputCurrent());
    // This method will be called once per scheduler run
  }
}
