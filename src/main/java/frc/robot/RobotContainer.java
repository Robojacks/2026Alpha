// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
// import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollersSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisonSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  public final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final VisonSubsystem m_visionSubsystem = new VisonSubsystem(m_driveSubsystem);
  // private final PhotonVisionSubsystem m_photonVisionSubsystem =
  // new PhotonVisionSubsystem(m_driveSubsystem);
  // private final ExampleSubsystem exampleSubsystem;
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IntakePivotSubsystem intakePivotSubsystem = new IntakePivotSubsystem();
  private final ShooterFeederSubsystem shooterFeederSubsystem = new ShooterFeederSubsystem();
  //   private final AgitatorSubsystem agitatorSubsystem = new AgitatorSubsystem();
  private final RollersSubsystem rollersSubsystem = new RollersSubsystem();

  // Controller
  // private final CommandXboxController controller = new CommandXboxController(0);
  // private final Joystick m_Joystick0 = new Joystick((OIConstants.kDriverControllerPort));
  // private final Joystick m_Joystick1 = new Joystick((OIConstants.kOperatorControllerPort));
  private final Translation2d m_fixedTarget;

  private final CommandXboxController theController =
      new CommandXboxController((OIConstants.kDriverControllerPort));
  private final CommandXboxController operatorController =
      new CommandXboxController((OIConstants.kOperatorControllerPort));

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("Auto Shooting Long", shooterSubsystem.autoShooterLongCommand());
    NamedCommands.registerCommand("Auto Shooting", shooterSubsystem.autoShooterCommand());
    NamedCommands.registerCommand("Auto Intake", intakeSubsystem.autoIntakeCommand());
    NamedCommands.registerCommand(
        "Auto IntakePivot up", intakePivotSubsystem.autoIntakePivotUpCommand());
    NamedCommands.registerCommand(
        "Auto IntakePivot down", intakePivotSubsystem.autoIntakePivotDownCommand());

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      m_fixedTarget =
          new Translation2d(
              Units.inchesToMeters(469.5),
              Units.inchesToMeters(158.32)); // Replace with actual red target coordinates
    } else {
      m_fixedTarget = new Translation2d(Units.inchesToMeters(181.55), Units.inchesToMeters(158.32));
    }

    m_visionSubsystem.dummyMethod(); // Call a dummy method to prevent unused variable warning

    // Configure the button bindings first so named commands are registered before building autos
    configureButtonBindings();

    // Set up auto routines (build chooser after named commands are registered)
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // PID setponts

    // new JoystickButton(m_Joystick0, 11).onTrue(intakePivotSubsystem.setOutCommand());

    // new JoystickButton(m_Joystick0, 12).onTrue(intakePivotSubsystem.setStowCommand());
    //  m_photonVisionSubsystem.setDefaultCommand(new RunCommand(() -> {},
    // m_photonVisionSubsystem));
    m_driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> {
              m_driveSubsystem.drive(
                  -MathUtil.applyDeadband(theController.getLeftY(), OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(theController.getLeftX(), OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(theController.getRightX(), OIConstants.kDriveDeadband),
                  true);
            },
            m_driveSubsystem));
    // SHOOT COMMAND
    // if (m_XboxController.getRawButton(0))
    operatorController
        .rightTrigger()
        .whileTrue(
            new ParallelCommandGroup(
                new RunCommand(
                    () -> shooterSubsystem.setPIDSpeed(Constants.ShooterConstants.shooterTargetRpm),
                    shooterSubsystem),
                new WaitCommand(0.75)
                    .andThen(
                        new RunCommand(
                            () ->
                                shooterFeederSubsystem.setShooterFeederSpeed(
                                    Constants.ShooterFeederConstants.shooterFeederSpeed),
                            shooterFeederSubsystem)),
                new WaitCommand(1.5)
                    .andThen(
                        new RunCommand(
                            () ->
                                rollersSubsystem.setRollersSpeed(
                                    Constants.RollersConstants.rollersSpeed),
                            rollersSubsystem))))
        .whileFalse(
            new ParallelCommandGroup(
                new RunCommand(
                    () -> shooterFeederSubsystem.setShooterFeederSpeed(0), shooterFeederSubsystem),
                new RunCommand(() -> shooterSubsystem.setShooterSpeed(0), shooterSubsystem),
                // new RunCommand(() -> agitatorSubsystem.setAgitatorSpeed(0), agitatorSubsystem)
                new RunCommand(() -> rollersSubsystem.setRollersSpeed(0), rollersSubsystem)));

    // RUN INTAKE
    operatorController
        .leftTrigger()
        .whileTrue(
            new RunCommand(
                () -> intakeSubsystem.setIntakeSpeed(Constants.IntakeConstants.intakeSpeed),
                intakeSubsystem))
        .whileFalse(new RunCommand(() -> intakeSubsystem.setIntakeSpeed(0), intakeSubsystem));

    // INTAKE PIVOT (operator D-pad)
    operatorController
        .povUp()
        .whileTrue(
            new RunCommand(
                () ->
                    intakePivotSubsystem.setIntakePivotSpeed(
                        -Constants.IntakePivotConstants.intakePivotSpeed),
                intakePivotSubsystem));

    operatorController
        .povDown()
        .whileTrue(
            new RunCommand(
                () ->
                    intakePivotSubsystem.setIntakePivotSpeed(
                        Constants.IntakePivotConstants.intakePivotSpeed),
                intakePivotSubsystem));

    operatorController
        .povUp()
        .or(operatorController.povDown())
        .whileFalse(
            new RunCommand(
                () -> intakePivotSubsystem.setIntakePivotSpeed(0), intakePivotSubsystem));

    // DEFENSE X-LOCK (hold X to lock swerve modules in an X pattern)
    theController.x().whileTrue(new RunCommand(() -> m_driveSubsystem.setX(), m_driveSubsystem));

    // ZERO HEADING
    theController
        .start()
        .whileTrue(new InstantCommand(() -> m_driveSubsystem.zeroHeading(), m_driveSubsystem));

    // Auto Aim
    // new JoystickButton(m_Joystick1, 10)
    //    .whileTrue(new TurnToTargetCommand(m_driveSubsystem, m_fixedTarget));
    // .onFalse(new InstantCommand(() -> m_driveSubsystem.stop(), m_driveSubsystem));

    // UNJAMING
    operatorController
        .leftBumper()
        .whileTrue(
            new ParallelCommandGroup(
                new RunCommand(
                    () ->
                        shooterSubsystem.setShooterSpeed(
                            -Constants.ShooterConstants.shooterSpeedPercent),
                    shooterSubsystem),
                new RunCommand(
                    () ->
                        rollersSubsystem.setRollersSpeed(-Constants.RollersConstants.rollersSpeed),
                    rollersSubsystem),
                new RunCommand(
                    () ->
                        shooterFeederSubsystem.setShooterFeederSpeed(
                            -Constants.ShooterFeederConstants.shooterFeederSpeed),
                    shooterFeederSubsystem)))
        .whileFalse(
            new ParallelCommandGroup(
                new RunCommand(() -> shooterSubsystem.setShooterSpeed(0), shooterSubsystem),
                new RunCommand(() -> rollersSubsystem.setRollersSpeed(0), rollersSubsystem),
                new RunCommand(
                    () -> shooterFeederSubsystem.setShooterFeederSpeed(0),
                    shooterFeederSubsystem)));

    // PID bution

    /*new JoystickButton(m_Joystick1, 12)
    .onTrue(
        new RunCommand(
            () -> shooterSubsystem.setPIDSpeed(Constants.ShooterConstants.shooterTargetRpm),
            shooterSubsystem))
    .onFalse(
        new RunCommand(
            () -> shooterSubsystem.setPIDSpeed(Constants.ShooterConstants.shooterIdleRpm),
            shooterSubsystem)); */

    // run auto for robot
    // run auto for robot
    NamedCommands.registerCommand(
        "Auto Shooting",
        Commands.parallel(
            shooterSubsystem.autoShooterCommand(),
            // agitatorSubsystem.autoAgitatorCommand(),
            shooterFeederSubsystem.autoShooterFeederCommand(),
            rollersSubsystem.autoRollersCommand()));

    NamedCommands.registerCommand(
        "Auto Shooting Long",
        Commands.parallel(
            shooterSubsystem.autoShooterLongCommand(),
            // agitatorSubsystem.autoAgitatorCommand(),
            shooterFeederSubsystem.autoShooterFeederLongCommand(),
            rollersSubsystem.autoRollersLongCommand()));

    NamedCommands.registerCommand(
        "Auto Intake", Commands.sequence(intakeSubsystem.autoIntakeCommand()));

    NamedCommands.registerCommand(
        "Auto IntakePivot up", Commands.sequence(intakePivotSubsystem.autoIntakePivotUpCommand()));
    NamedCommands.registerCommand(
        "Auto IntakePivot down",
        Commands.sequence(intakePivotSubsystem.autoIntakePivotDownCommand()));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
