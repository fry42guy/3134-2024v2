// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmPivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ArmSubsystem;

public class PIDPivotCommand extends CommandBase {
  /** Creates a new PIDPivotCommand. */
  private PIDController m_PivotPIDController;
  private final ArmSubsystem m_ArmSubsystem;
  private double setPoint;
  private boolean m_UseCurrentPosition;
  public PIDPivotCommand(ArmSubsystem m_ArmSubsystem, double setPoint, boolean UseCurrentPosition) {
    this.m_ArmSubsystem = m_ArmSubsystem;
    m_PivotPIDController = new PIDController(.00004, 0., 0.0);
   // m_PivotPIDController.enableContinuousInput(-1, 1);
    m_PivotPIDController.setTolerance(0.0035);
    this.setPoint = setPoint;
    this.m_UseCurrentPosition = UseCurrentPosition;
    addRequirements(m_ArmSubsystem);

    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (m_UseCurrentPosition){
setPoint = m_ArmSubsystem.getPivotEncoder();
    }



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double feedforward = 0.00;
    double speed = m_PivotPIDController.calculate(m_ArmSubsystem.getPivotEncoder(), setPoint);
    speed = (speed > 0) ? speed + feedforward : speed - feedforward;
    m_ArmSubsystem.setspeed(speed);
    SmartDashboard.putNumber("Pivot output: ", speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_ArmSubsystem.setspeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_PivotPIDController.atSetpoint())
          return true;
    return false;

  }

  public void setPoint(double setPoint)
  {
    this.setPoint = setPoint;
  }
}
