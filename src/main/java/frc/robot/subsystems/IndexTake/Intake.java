package frc.robot.subsystems.IndexTake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  public static final CANBus kCANBus = new CANBus("CTRE Devs");

  // Those Who Declare

  private final TalonFX armMotor = new TalonFX(IndexTakeConstants.intakeMoveCanId, kCANBus);
  private final TalonFX spinMotor = new TalonFX(IndexTakeConstants.intakeSpinCanId, kCANBus);
  private final TalonFXConfiguration armConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration spinConfig = new TalonFXConfiguration();

  private double armOffset = 0; // needs to be tuned

  // Constants (untuned)
  private static double kP = 0;
  private static double kD = 0;
  private static double kG = 0;
  private static double kS = 0;
  private final ArmFeedforward armFeedForward = new ArmFeedforward(kS, kG, 0);
  private final PIDController armController = new PIDController(kP, 0, kD);

  // Variables
  private static final double arm_Position_Conversion_Factor = 1 / 20;

  // Offsets
  private double tunablekP = 0.0;
  private double tunablekD = 0.0;
  private double tunablekG = 0.0;

  public Intake() {
    armConfig.Feedback.SensorToMechanismRatio = arm_Position_Conversion_Factor;
    armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armConfig.Slot0.kG = .9575;
    armConfig.Slot0.kS = .5425;
    armConfig.Slot0.kP = 1;
    armConfig.Slot0.kD = .001;
    armConfig.MotionMagic.MotionMagicAcceleration = 100;
    armConfig.MotionMagic.MotionMagicCruiseVelocity = 50;
    armConfig.MotionMagic.MotionMagicJerk = 500;

    spinConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    spinConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    armMotor.getConfigurator().apply(armConfig);
    spinMotor.getConfigurator().apply(spinConfig);

    instantiateTunables();

    setArmOffset();
  }

  public void setArmOffset() {
    armMotor.setVoltage(-.2);
    armMotor.setPosition(armOffset);
  }

  public double getArmPosition() {

    return armMotor.getPosition().getValueAsDouble();
  }

  public void instantiateTunables() {
    SmartDashboard.putNumber("Intake/Pivot/kP", tunablekP);
    SmartDashboard.putNumber("Intake/Pivot/kD", tunablekD);
    SmartDashboard.putNumber("Intake/Pivot/kG", tunablekG);
    SmartDashboard.putNumber("Intake/Pivot/ArmPosition", getArmPosition());
  }

  public Command manualArmMove(double voltage) {
    return this.runEnd(
        () -> armMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(voltage)),
        () -> {
          armMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0));
        });
  }

  public Command intakeArmStop() {
    return this.runOnce(() -> armMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0)));
  }

  public Command intakeSpin(double voltage) {
    return this.runEnd(
        () -> spinMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(voltage)),
        () -> spinMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0)));
  }

  public FunctionalCommand runIntake(double voltage) {
    return new FunctionalCommand(
        () -> {
          spinMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(voltage));
        },
        () -> {},
        (interrupted) -> {
          spinMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0));
        },
        () -> false);
  }

  public Command rawMoveIntake(double voltage) {
    return this.runEnd(
        () -> {
          armMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(voltage));
        },
        () -> {
          armMotor.setControl(new VoltageOut(0));
        });
  }

  @Override
  public void periodic() {
    updateValues();
  }

  public void updateValues() {
    double currentkP = tunablekP;
    double currentkD = tunablekD;
    double currentkG = tunablekG;
    double currentPosition = getArmPosition();
    double putArmPosition = currentPosition;

    tunablekP = SmartDashboard.getNumber("Intake/Pivot/kP", tunablekP);
    tunablekD = SmartDashboard.getNumber("Intake/Pivot/kD", tunablekD);
    tunablekG = SmartDashboard.getNumber("Intake/Pivot/kG", tunablekG);
    putArmPosition = SmartDashboard.getNumber("Intake/Pivot/ArmPosition", currentPosition);

    if (currentPosition != putArmPosition) {
      armMotor.setPosition(putArmPosition);
    }
  }
}
