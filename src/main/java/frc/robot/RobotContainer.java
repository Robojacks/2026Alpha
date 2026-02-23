// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  // private final ExampleSubsystem exampleSubsystem;
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IntakePivotSubsystem intakePivotSubsystem = new IntakePivotSubsystem();
  private final ShooterFeederSubsystem shooterFeederSubsystem = new ShooterFeederSubsystem();
  private final AgitatorSubsystem agitatorSubsystem = new AgitatorSubsystem();

  // Controller
  // private final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick m_Joystick0 = new Joystick(DriveConstants.kDriverControllerPort0);
  private final Joystick m_Joystick1 = new Joystick(DriveConstants.kDriverControllerPort1);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        //  exampleSubsystem = new ExampleSubsystem();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        //    exampleSubsystem = new ExampleSubsystem();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // exampleSubsystem = new ExampleSubsystem();
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    /*drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .onTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));*/

    // Configure default commands
    /*drive.setDefaultCommand(
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    new RunCommand(
        () -> drive.drive(
            -MathUtil.applyDeadband(m_Joystick0.getY() * 0.25, DriveConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_Joystick0.getX() * 0.25, DriveConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_Joystick1.getX(), DriveConstants.kDriveDeadband),
            true),
        drive));*/

    /*   drive.setDefaultCommand(
    DriveCommands.joystickDrive(
        drive,
        () -> -MathUtil.applyDeadband(m_Joystick0.getY() * 0.25, DriveConstants.kDriveDeadband),
        () -> -MathUtil.applyDeadband(m_Joystick0.getX() * 0.25, DriveConstants.kDriveDeadband),
        () -> -MathUtil.applyDeadband(m_Joystick1.getX(), DriveConstants.kDriveDeadband)));
        */
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> m_Joystick0.getY(), () -> m_Joystick0.getX(), () -> m_Joystick1.getX()));

    /*new JoystickButton(m_Joystick1, 1)
    .onTrue(
        new RunCommand(() -> shooterSubsystem.setShooterSpeed(0.5), shooterSubsystem)
            .andThen(new WaitCommand(0.5))
            .andThen(
                () -> shooterFeederSubsystem.setShooterFeederSpeed(0.5), shooterFeederSubsystem)
            .andThen(() -> agitatorSubsystem.setAgitatorSpeed(0.5), agitatorSubsystem));*/
    new JoystickButton(m_Joystick1, 1)
        .onTrue(
            new RunCommand(
                () -> shooterSubsystem.setShooterSpeed(Constants.ShooterConstants.shooterSpeed),
                shooterSubsystem))
        .onFalse(new RunCommand(() -> shooterSubsystem.setShooterSpeed(0), shooterSubsystem));

    new JoystickButton(m_Joystick0, 6)
        .onTrue(
            new RunCommand(
                () -> intakeSubsystem.setIntakeSpeed(Constants.IntakeConstants.intakeSpeed),
                intakeSubsystem))
        .onFalse(new RunCommand(() -> intakeSubsystem.setIntakeSpeed(0), intakeSubsystem));

    new JoystickButton(m_Joystick0, 1)
        .onTrue(
            new RunCommand(
                    () ->
                        shooterFeederSubsystem.setShooterFeederSpeed(
                            Constants.ShooterFeederConstants.shooterFeederSpeed),
                    shooterFeederSubsystem)
                .alongWith(
                    new RunCommand(
                        () ->
                            agitatorSubsystem.setAgitatorSpeed(
                                -Constants.AgitatorConstants.agitatorSpeed))))
        .onFalse(
            new RunCommand(() -> agitatorSubsystem.setAgitatorSpeed(0), agitatorSubsystem)
                .alongWith(
                    new RunCommand(
                        () -> shooterFeederSubsystem.setShooterFeederSpeed(0),
                        shooterFeederSubsystem)));

    new JoystickButton(m_Joystick1, 5)
        .onTrue(
            new RunCommand(
                () ->
                    intakePivotSubsystem.setIntakePivotSpeed(
                        Constants.IntakePivotConstants.intakePivotSpeed),
                intakePivotSubsystem))
        .onFalse(
            new RunCommand(
                () -> intakePivotSubsystem.setIntakePivotSpeed(0), intakePivotSubsystem));

    new JoystickButton(m_Joystick1, 3)
        .onTrue(
            new RunCommand(
                () ->
                    intakePivotSubsystem.setIntakePivotSpeed(
                        -Constants.IntakePivotConstants.intakePivotSpeed),
                intakePivotSubsystem))
        .onFalse(
            new RunCommand(
                () -> intakePivotSubsystem.setIntakePivotSpeed(0), intakePivotSubsystem));

    new JoystickButton(m_Joystick0, 5)
        .onTrue(
            new RunCommand(
                () ->
                    shooterFeederSubsystem.setShooterFeederSpeed(
                        -Constants.ShooterFeederConstants.shooterFeederSpeed),
                shooterFeederSubsystem))
        .onFalse(new RunCommand(() -> shooterFeederSubsystem.setShooterFeederSpeed(0)));

    /*new JoystickButton(m_Joystick0, 1)
    .onTrue(new RunCommand(
        () -> m_robotDrive.setX(),
        m_robotDrive));

    // Run example motor at set speed when Y button is held
    /*controller
        .y()
        .onTrue(Commands.run(() -> exampleSubsystem.motor1.set(0.5), exampleSubsystem))
        .onFalse(Commands.run(() -> exampleSubsystem.motor1.set(0), exampleSubsystem));

    controller.x().onTrue(exampleSubsystem.testCommand());
    controller.x().whileFalse(exampleSubsystem.testCommand2());*/
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
