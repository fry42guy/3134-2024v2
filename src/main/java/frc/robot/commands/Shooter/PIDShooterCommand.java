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


  public PIDShooterCommand(ShooterSubsystem m_ShooterSubsystem) {
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    m_LeftShooterPIDController = new PIDController(.00004, 0., 0.0);
   // m_ShooterPIDController.enableContinuousInput(-1, 1);
    m_LeftShooterPIDController.setTolerance(0.0035);
    
   
    addRequirements(m_ShooterSubsystem);

    

    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_ShooterSubsystem.PIDSetup();

    m_ShooterSubsystem.PIDUpdate();

    m_LeftShooterPIDController.setP(m_ShooterSubsystem.KP);
    m_LeftShooterPIDController.setI(m_ShooterSubsystem.KI);
    m_LeftShooterPIDController.setD(m_ShooterSubsystem.KD);
    //System.out.println("PID init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double feedforward = 0.00;
    double speed = m_LeftShooterPIDController.calculate(m_ShooterSubsystem.GetLeftShooterRPM(), m_ShooterSubsystem.PIDTESTspeed);
    speed = (speed > 0) ? speed + feedforward : speed - feedforward;
    m_ShooterSubsystem.setspeed(speed);
    SmartDashboard.putNumber("LeftShooter output: ", speed);
    //System.out.println(m_ShooterSubsystem.GetLeftShooterRPM());
    //System.out.println(LeftsetPoint);
    //System.out.println(speed);
   //System.out.println(KP);
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
