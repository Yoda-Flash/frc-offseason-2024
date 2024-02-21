// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Constants.WristConstants;
// import frc.robot.subsystems.Wrist;

// public class RotateWrist extends Command {
//   private double m_endPosition;
//   private Wrist m_wrist;
//   private PIDController m_PID = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
//   /** Creates a new RotatePivot. */
//   public RotateWrist(double angle, Wrist wrist) {
//     m_wrist = new Wrist();
//     m_endPosition = angle*Constants.kFalconTicksPerRevolution*WristConstants.kGearRatio/360;
//   }
//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_wrist.setSpeed(m_PID.calculate(m_wrist.getEncoderPosition(), m_endPosition));
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_wrist.setSpeed(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return (Math.abs(m_wrist.getEncoderPosition() - m_endPosition) < Constants.WristConstants.kDeadBand);
//   }
// }
