// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private final TalonFX LeftPivot;
  private final TalonFX RightPivot;
  private final DutyCycleEncoder HexEncoder;
  


  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    LeftPivot = new TalonFX(Constants.Arm.LeftPivotID);
    LeftPivot.setInverted(false);
    RightPivot = new TalonFX(Constants.Arm.RightPivotID);
    RightPivot.setInverted(false);
    HexEncoder = new DutyCycleEncoder(Constants.Arm.EncoderPWMID);



  }

  public void setspeed(Double speed){
LeftPivot.set(ControlMode.PercentOutput, speed);
RightPivot.set(ControlMode.PercentOutput, speed);

  }

  public void stop(){
    LeftPivot.set(ControlMode.PercentOutput, 0);
    RightPivot.set(ControlMode.PercentOutput, 0);
    
      }



  @Override
  public void periodic() {

    SmartDashboard.putNumber("Encoder Value", HexEncoder.getAbsolutePosition()*360);
    // This method will be called once per scheduler run
  }
}
