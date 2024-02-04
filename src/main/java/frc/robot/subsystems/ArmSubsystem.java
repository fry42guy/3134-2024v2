// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private final TalonFX PivotMotor;
  
  private final DutyCycleEncoder HexEncoder;

  private final double MaxCurrent;
  


  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    PivotMotor = new TalonFX(Constants.Arm.LeftPivotID);
    PivotMotor.setInverted(false);
   
    HexEncoder = new DutyCycleEncoder(Constants.Arm.EncoderPWMID);
    MaxCurrent = Constants.Arm.MAX_CURRENT_DRAW;

    ConfigPivotCurrent();

  }

  public void ConfigPivotCurrent(){

    if (MaxCurrent >= 5){
      PivotMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,Constants.Arm.MAX_CURRENT_DRAW,Constants.Arm.MAX_CURRENT_DRAW + 5, 0.5));
      }
      else{
        PivotMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false,40,10,1));
      
      }

  }

  public void setspeed(Double speed){
PivotMotor.set(ControlMode.PercentOutput, speed);


  }

  public void stop(){
    PivotMotor.set(ControlMode.PercentOutput, 0);
  
    
      }

      public double getPivotEncoder(){

       return PivotMotor.getSelectedSensorPosition();

      } 



  @Override
  public void periodic() {

    SmartDashboard.putNumber("Hex Encoder Value", HexEncoder.getAbsolutePosition()*360);
    SmartDashboard.putNumber("Pivot Motor Encoder", getPivotEncoder());
    SmartDashboard.putNumber("Pivot Motor Current",PivotMotor.getStatorCurrent());

    // This method will be called once per scheduler run
  }
}
