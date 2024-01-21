// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class ShooterSubsystem extends SubsystemBase {

private final TalonFX LeftShooter;
private final TalonFX RightShooter;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    LeftShooter = new TalonFX(Constants.Shooter.LeftShooterID);
    LeftShooter.setInverted(true);
    LeftShooter.setSelectedSensorPosition(0);
    LeftShooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0,25);

    RightShooter = new TalonFX(Constants.Shooter.RightShooterID);
    RightShooter.setInverted(true);
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



  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
