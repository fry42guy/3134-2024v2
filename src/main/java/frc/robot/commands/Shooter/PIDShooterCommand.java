// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ShooterSubsystem;

public class PIDShooterCommand extends CommandBase {
  /** Creates a new PIDShooterCommand. */
  private PIDController m_LeftShooterPIDController;
  private final ShooterSubsystem m_ShooterSubsystem;
  private double LeftsetPoint;
  

private double KP;
private double KI;
private double KD;


  public PIDShooterCommand(ShooterSubsystem m_ShooterSubsystem, double TargetRPM) {
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    m_LeftShooterPIDController = new PIDController(.00004, 0., 0.0);
   // m_ShooterPIDController.enableContinuousInput(-1, 1);
    m_LeftShooterPIDController.setTolerance(0.0035);
    this.LeftsetPoint = TargetRPM;
   
    addRequirements(m_ShooterSubsystem);

    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

SmartDashboard.setDefaultNumber("LeftShooter(PID) KP", 0.00004);
SmartDashboard.setDefaultNumber("LeftShooter(PID) KI", 0.0);
SmartDashboard.setDefaultNumber("LeftShooter(PID) KD", 0.0);
SmartDashboard.setPersistent("LeftShooter(PID) KP");
SmartDashboard.setPersistent("LeftShooter(PID) KI");
SmartDashboard.setPersistent("LeftShooter(PID) KD");
    SmartDashboard.getNumber("LeftShooter(PID) KP", KP);
    SmartDashboard.getNumber("LeftShooter(PID) KI", KI);
    SmartDashboard.getNumber("LeftShooter(PID) KD", KD);
    m_LeftShooterPIDController.setP(KP);
    m_LeftShooterPIDController.setI(KI);
    m_LeftShooterPIDController.setD(KD);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double feedforward = 0.00;
    double speed = m_LeftShooterPIDController.calculate(m_ShooterSubsystem.GetLeftShooterRPM(), LeftsetPoint);
    speed = (speed > 0) ? speed + feedforward : speed - feedforward;
    m_ShooterSubsystem.setspeed(speed);
    SmartDashboard.putNumber("LeftShooter output: ", speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_ShooterSubsystem.setspeed(0.0);////////////////////////
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  
    return false;

  }

  public void LeftsetPoint(double LeftsetPoint)
  {
    this.LeftsetPoint = LeftsetPoint;
  }
}
