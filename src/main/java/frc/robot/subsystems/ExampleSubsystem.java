// package frc.robot.subsystems;

// import com.revrobotics.PersistMode;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class ExampleSubsystem extends SubsystemBase {

//   public SparkMax motor1 = new SparkMax(99, MotorType.kBrushless);

//   public ExampleSubsystem() {
//     SparkMaxConfig motor1Config = new SparkMaxConfig();
//     motor1Config.smartCurrentLimit(50);
//     motor1Config.idleMode(IdleMode.kBrake);
//     motor1.configure(
//         motor1Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
//     // motor1.setIdleMode(SparkMax.IdleMode.kBrake);
//   }

//   public Command testCommand() {
//     return Commands.print("XXXXX X BUTTON PUSHED XXXXX");
//   }

//   public Command testCommand2() {
//     return Commands.print("XXXXX X BUTTON NOT PUSHED XXXXX");
//   }
// }
