// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class ShooterSubsystem extends SubsystemBase {

private final TalonFX LeftShooter;
private final TalonFX RightShooter;

public double KP;
public double KI;
public double KD;
public double PIDTESTspeed;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    LeftShooter = new TalonFX(Constants.Shooter.LeftShooterID);
    LeftShooter.setInverted(false);
    LeftShooter.setSelectedSensorPosition(0);
    LeftShooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0,25);

    RightShooter = new TalonFX(Constants.Shooter.RightShooterID);
    RightShooter.setInverted(false);
    RightShooter.setSelectedSensorPosition(0);
    RightShooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0,25);
    

  }

public void stop(){

  LeftShooter.set(ControlMode.PercentOutput, 0);
  RightShooter.set(ControlMode.PercentOutput, 0);

}

public void setspeed(Double speed){

LeftShooter.set(ControlMode.PercentOutput, speed);
RightShooter.set(ControlMode.PercentOutput, speed);


}

public void setDiffSpeed(Double leftspeed, double rightspeed){

  LeftShooter.set(ControlMode.PercentOutput, leftspeed);
  RightShooter.set(ControlMode.PercentOutput, rightspeed);

}

public double GetRightShooterRPM(){


  return (RightShooter.getSelectedSensorVelocity() / 2048) * 10 * 60;

}

public double GetLeftShooterRPM(){


  return (LeftShooter.getSelectedSensorVelocity() / 2048) * 10 * 60;

}

public void PIDSetup(){

if (SmartDashboard.containsKey("LeftShooter(PID) KP")==false){
SmartDashboard.putNumber("LeftShooter(PID) KP", 0.0004);
}

if (SmartDashboard.containsKey("LeftShooter(PID) KI") ==false){
  SmartDashboard.putNumber("LeftShooter(PID) KI", 0.0);
  }

  if (SmartDashboard.containsKey("LeftShooter(PID) KD")==false){
    SmartDashboard.putNumber("LeftShooter(PID) KD", 0.0);
    }

    if (SmartDashboard.containsKey("TEST PID (RPM) Speed")==false){
      SmartDashboard.putNumber("TEST PID (RPM) Speed", 2500);
      }



}

public void PIDUpdate(){

  KP = SmartDashboard.getNumber("LeftShooter(PID) KP", 0.0);
  KI = SmartDashboard.getNumber("LeftShooter(PID) KI", 0.0);
  KD = SmartDashboard.getNumber("LeftShooter(PID) KD", 0.0);
  PIDTESTspeed = SmartDashboard.getNumber("TEST PID (RPM) Speed", 0.0);

  if (PIDTESTspeed > 5000){

    PIDTESTspeed = 5000;
    SmartDashboard.putNumber("TEST PID (RPM) Speed", PIDTESTspeed);
  }
  if (PIDTESTspeed < -5000){

    PIDTESTspeed = -5000;
    SmartDashboard.putNumber("TEST PID (RPM) Speed", PIDTESTspeed);
  }


  SmartDashboard.setPersistent("LeftShooter(PID) KP");
  SmartDashboard.setPersistent("LeftShooter(PID) KI");
  SmartDashboard.setPersistent("LeftShooter(PID) KD");
  SmartDashboard.setPersistent("TEST PID (RPM) Speed");
}
  @Override
  public void periodic() {

    SmartDashboard.putNumber("Left Shooter Motor RPM", GetLeftShooterRPM());
    SmartDashboard.putNumber("Right Shooter Motor RPM ", GetRightShooterRPM());


    // This method will be called once per scheduler run
  }
}
