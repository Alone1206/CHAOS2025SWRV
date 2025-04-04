// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeMove;
/* import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.MotorStopSubsystem;
*/
// import frc.robot.subsystems.vision;
import frc.robot.subsystems.ShooterSubsystem; 
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
   private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
   private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
   private final IntakeMove intakeSubsystem = new IntakeMove();
   // Add the shooter subsystem
 /* private final CoralSubsystem CoralSubsystem = new CoralSubsystem(); // Add the coral subsystem
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(); // Add the elevator subsystem
  private final MotorStopSubsystem motorStopSubsystem = new MotorStopSubsystem(); // Add the motor stop subsystem */
   //private final vision vision = new vision(); // Add the motor stop subsystem */

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick m_driverController =
      new CommandJoystick(OperatorConstants.kDriverControllerPort);

  private final CommandJoystick m_operatorController =
      new CommandJoystick(OperatorConstants.kOperatorControllerPort);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /* NamedCommands.registerCommand("shoot", shooterSubsystem.ShootBall());
    NamedCommands.registerCommand("coral", CoralSubsystem.CoralDrop()); */
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedAngularVelocity : driveFieldOrientedDirectAngleSim);
  }

  //SURUS ICIN GEREKLI OLAN KODLAR

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getRawAxis(1), // Left Y axis
                                                                () -> m_driverController.getRawAxis(0))    // Left X axis
                                                                .withControllerRotationAxis(() -> m_driverController.getRawAxis(2)) // Right X axis
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(0.8)
                                                                .allianceRelativeControl(false);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> m_driverController.getRawAxis(2), //Right X axis
                                                                                             () -> m_driverController.getRawAxis(3)) //Right Y axis
                                                                                             .headingWhile(true);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  // SIMULASYON
  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                                          () -> m_driverController.getRawAxis(1), // Left Y axis
                                                                                          () -> m_driverController.getRawAxis(0)) // Left X axis
                                                                                          .withControllerRotationAxis(() -> m_driverController.getRawAxis(2)) 
                                                                                          .deadband(OperatorConstants.DEADBAND)
                                                                                          .scaleTranslation(0.8)
                                                                                          .allianceRelativeControl(true);         
                                                                                          
  SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
                                                                      .withControllerHeadingAxis(() -> Math.sin(m_driverController.getRawAxis(2) * Math.PI * (Math.PI *2)),
                                                                                                 () -> Math.cos(m_driverController.getRawAxis(2) * Math.PI * (Math.PI *2)))
                                                                                                       .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

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
  //  new Trigger(() -> m_driverController.getHID().getRawButton(8))
  //   .onTrue(new InstantCommand(() -> shooterSubsystem.shootBall()));
   double axis1 = m_operatorController.getRawAxis(1);
   new Trigger(() -> m_operatorController.getRawAxis(1) > 0.08 || m_operatorController.getRawAxis(1) < -0.08)
   .whileTrue(new RunCommand(() -> shooterSubsystem.shootBall(m_operatorController.getRawAxis(1))))
   .onFalse(new RunCommand(() -> shooterSubsystem.shootBall(0))); // Stop motor when joystick is released

   new Trigger(() -> m_operatorController.getHID().getRawButton(4))
   .whileTrue(new RunCommand(() -> elevatorSubsystem.elevatorMoveUp()))
   .onFalse(new RunCommand(() -> elevatorSubsystem.elevatorStop())); // Stop motor when joystick is released

   new Trigger(() -> m_operatorController.getHID().getRawButton(2))
   .whileTrue(new RunCommand(() -> elevatorSubsystem.elevatorMoveDown()))
   .onFalse(new RunCommand(() -> elevatorSubsystem.elevatorStop())); // Stop motor when joystick is released

   new Trigger(() -> m_operatorController.getHID().getRawButton(3))
   .whileTrue(new RunCommand(() -> intakeSubsystem.intakeMotorMoveFront()))
   .onFalse(new RunCommand(() -> intakeSubsystem.intakeStop())); // Stop motor when joystick is released

   new Trigger(() -> m_operatorController.getHID().getRawButton(1))
   .whileTrue(new RunCommand(() -> intakeSubsystem.intakeMotorMoveRear()))
   .onFalse(new RunCommand(() -> intakeSubsystem.intakeStop())); // Stop motor when joystick is released
// double axis1 = m_operatorController.getRawAxis(1);
// boolean is_on = m_operatorController.getHID().getRawButton(6);
// System.out.println(is_on);
//      if (m_operatorController.getHID().getRawButton(6)){
//       System.out.println("patates");

//       // shooterSubsystem.shootBall(0.5);
//     } 
   /*   
    else if (m_operatorController.getHID().getRawButton(6)){
      elevatorSubsystem.levelDown();
    }

    if (m_operatorController.getHID().getRawButton(7)){
      CoralSubsystem.coralDrop(0.8);
    } else if (m_operatorController.getHID().getRawButton(8)){
      CoralSubsystem.coralDrop(-0.8);;
    }

    if (m_operatorController.getHID().getRawButton(8)){
      shooterSubsystem.shootBall(0.8);
    } else if (m_operatorController.getHID().getRawButton(9)){
      shooterSubsystem.shootBall(-0.8);
    } */
  }
  


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("right");
  }
}