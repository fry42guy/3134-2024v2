// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Intake;
import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ArmPivot.ArmDown;
import frc.robot.commands.ArmPivot.ArmUP;
import frc.robot.commands.ArmPivot.PIDPivotCommand;
import frc.robot.commands.Intake.IntakeFWD;
import frc.robot.commands.Intake.IntakeREV;
import frc.robot.commands.Shooter.PIDShooterCommand;
import frc.robot.commands.Shooter.ShootSpeedSame;
import frc.robot.subsystems.ArmSubsystem;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

      private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    
  public static Swerve s_Swerve = new Swerve();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
      new TeleopSwerve(
          s_Swerve, 
          // () -> -modifyAxis(m_Drive_Controller.getLeftY()*.8),// * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          // () -> -modifyAxis(m_Drive_Controller.getLeftX()*.8), //* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          // () -> -modifyAxis(m_Drive_Controller.getRightX()*.7),// * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
 
          () -> m_driverController.getLeftY()*Math.abs(m_driverController.getLeftY())*-.9, 
          () -> m_driverController.getLeftX()*Math.abs(m_driverController.getLeftX())*-.9, 
          () -> m_driverController.getRightX()*Math.abs(m_driverController.getRightX())*-1, 
          () -> false //() -> robotCentric.getAsBoolean() //always field centric
      )
  );
 
   m_ArmSubsystem.setDefaultCommand(new PIDPivotCommand(m_ArmSubsystem,0,true));
    // Configure the trigger bindings
    configureBindings();
  }


  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    m_driverController.leftBumper().whileTrue(new ShootSpeedSame(m_ShooterSubsystem,Constants.Shooter.FastSpeed));
m_driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, .05).whileTrue(new ShootSpeedSame(m_ShooterSubsystem,Constants.Shooter.SlowSpeed));

    m_driverController.x().whileTrue(new ArmDown(m_ArmSubsystem)); /////////////////////// UPDATE to CORRECT BUTTONS????????????????????????????????????????
    m_driverController.y().whileTrue(new ArmUP(m_ArmSubsystem)); /////////////////////// UPDATE to CORRECT BUTTONS????????????????????????????????????????

    m_driverController.rightBumper().whileTrue(new IntakeFWD(m_IntakeSubsystem));
    m_driverController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .05).whileTrue(new IntakeREV(m_IntakeSubsystem));

m_driverController.a().whileTrue(new PIDShooterCommand(m_ShooterSubsystem));



//         // No requirements because we don't need to interrupt anything
       m_driverController.back().onTrue(new InstantCommand (()-> s_Swerve.zeroGyro()));
          


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
