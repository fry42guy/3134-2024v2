// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX IntakeMotor;
  /** Creates a new Intake. */
  public IntakeSubsystem() {

IntakeMotor = new TalonFX(Constants.Intake.IntakeMotorID);
IntakeMotor.setInverted(true);


  }

  public void setspeed(Double speed){
IntakeMotor.set(ControlMode.PercentOutput, speed);

  }

// public void IntakeFWD(){
//   IntakeMotor.set(ControlMode.PercentOutput, Constants.Intake.FWDSpeed);
// }

// public void IntakeREV(){
//   IntakeMotor.set(ControlMode.PercentOutput, Constants.Intake.REVSpeed);
// }

public void Stop(){

  IntakeMotor.set(ControlMode.PercentOutput, 0);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
