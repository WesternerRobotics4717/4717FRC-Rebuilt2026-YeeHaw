package frc.robot.subsystems.IndexTake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase {

  public static final CANBus kCANBus = new CANBus("CTRE Devs");

  // Those Who Declare

  private final TalonFX armMotor = new TalonFX(IndexTakeConstants.intakeMoveCanId, kCANBus);
  private final TalonFX spinMotor = new TalonFX(IndexTakeConstants.intakeSpinCanId, kCANBus);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private final TalonFXConfiguration armConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration spinConfig = new TalonFXConfiguration();

  private double armOffset = 0; // needs to be tuned

  // Constants (untuned)

  // Variables
  private static final double arm_Position_Conversion_Factor = 1 / 20;

  // Offsets
  private final LoggedNetworkNumber tunablekP = new LoggedNetworkNumber("Intake/Pivot/kP", 1.0);
  private final LoggedNetworkNumber tunablekD = new LoggedNetworkNumber("Intake/Pivot/kD", 0.001);
  private final LoggedNetworkNumber tunablekG = new LoggedNetworkNumber("Intake/Pivot/kG", 0.9575);
  private final LoggedNetworkNumber intakeSetpoint =
      new LoggedNetworkNumber("Intake/Pivot/Setpoint", 0);

  private double lastkP = 0.0;
  private double lastkD = 0.0;
  private double lastkG = 0.0;

  public Intake() {
    armConfig.Feedback.SensorToMechanismRatio = arm_Position_Conversion_Factor;
    armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armConfig.Slot0.kG = tunablekG.get();
    armConfig.Slot0.kS = .5425;
    armConfig.Slot0.kP = tunablekP.get();
    armConfig.Slot0.kD = tunablekD.get();
    armConfig.MotionMagic.MotionMagicAcceleration = 100;
    armConfig.MotionMagic.MotionMagicCruiseVelocity = 50;
    armConfig.MotionMagic.MotionMagicJerk = 500;

    spinConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    spinConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    armMotor.getConfigurator().apply(armConfig);
    spinMotor.getConfigurator().apply(spinConfig);

    setArmOffset();
  }

  public Command setArmOffset() {
    return Commands.run(() -> armMotor.setVoltage(-.3), this)
        .withTimeout(.75)
        .andThen(Commands.runOnce(() -> armMotor.setPosition(armOffset)));
  }

  public double getArmPosition() {
    return armMotor.getPosition().getValueAsDouble();
  }

  public Command setTunableArmPosition() {
    return this.run(
        () -> armMotor.setControl(motionMagicRequest.withPosition(intakeSetpoint.getAsDouble())));
  }

  public Command intakeArmStop() {
    return this.runOnce(() -> armMotor.setControl(new VoltageOut(0)));
  }

  // public Command moveArmToPosition() {
  // return this.run(() -> setArmPosition(newArmPosition));}

  public FunctionalCommand runIntake(double voltage) {
    return new FunctionalCommand(
        () -> {
          spinMotor.setControl(new VoltageOut(voltage));
        },
        () -> {},
        (interrupted) -> {
          spinMotor.setControl(new VoltageOut(0));
        },
        () -> false);
  }

  public FunctionalCommand rawMoveIntake(double voltage) {
    return new FunctionalCommand(
        () -> {
          armMotor.setControl(new VoltageOut(voltage));
        },
        () -> {},
        (interrupted) -> {
          armMotor.setControl(new VoltageOut(0));
        },
        () -> false);
  }

  @Override
  public void periodic() {
    updateValues();
  }

  public void updateValues() {
    double kP = tunablekP.get();
    double kD = tunablekD.get();
    double kG = tunablekG.get();

    if (kP != lastkP || kD != lastkD) {
      lastkP = kP;
      lastkD = kD;
      armMotor.getConfigurator().apply(armConfig);
    }

    if (kG != lastkG) {
      lastkG = kG;
      armConfig.Slot0.kG = kG;
      armMotor.getConfigurator().apply(armConfig);
    }
  }

  public Command moveArmDown() {
    return this.run(
        () -> {
          rawMoveIntake(0);
        });
  }
}
