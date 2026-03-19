package frc.robot.subsystems.IndexTake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  // Those Who Declare

  //TODO:Change to Krakenx44 system

  private final TalonFX armMoveMotor =
      new TalonFX(IndexTakeConstants.intakeMoveCanId);
  private final TalonFX spinIntakeMotor =
      new TalonFX(IndexTakeConstants.intakeSpinCanId);

 // private final RelativeEncoder relativeMoveEncoder = armMoveMotor.getEncoder();

 // private final SparkClosedLoopController armMovePID = armMoveMotor.getClosedLoopController();

  // Constants (untuned)
  private static double kP = 0;
  private static double kI = 0;
  private static double kD = 0;
  private static double kG = 0;
  private static double kS = 0;
  private final ArmFeedforward armFeedForward = new ArmFeedforward(kS, kG, 0);
  private final PIDController armController = new PIDController(kP, kI, kD);

  // Variables CUz  (need to be done, i dont rememb er them)
  private static final double arm_Position_Conversion_Factor = 60;

  // Offsets
  private static double startPosition = 0;

  private double tunablekP = 0.0;
  private double tunablekI = 0.0;
  private double tunablekD = 0.0;
  private double tunablekG = 0.0;
  private double tunableMaxAccel = 0.0;
  private double tunableMaxVelocity = 0.0;

  public Intake() {
    armMoveMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    spinIntakeMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Coast);
  }


  public Command manualArmMove(double voltage) {
    return this.runEnd(
        () -> armMoveMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(voltage)),
        () -> {
          armMoveMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0));
        });
  }

  public Command intakeArmStop() {
    return this.runOnce(() -> armMoveMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0)));
  }

  public Command intakeSpin(double voltage) {
    return this.runEnd(
        () -> spinIntakeMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(voltage)), () -> spinIntakeMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0)));
  }

  public FunctionalCommand runIntake(double voltage) {
    return new FunctionalCommand(
        () -> {
          spinIntakeMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(voltage));
        },
        () -> {},
        (interrupted) -> {
          spinIntakeMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0));
        },
        () -> false);
  }

  @Override
  public void periodic() {
  //SmartDashboard.putNumber("Intake/Pivot/Current Position ", getArmPositionDegrees());
  //SmartDashboard.putNumber("Intake/Pivot/Raw Position", relativeMoveEncoder.getPosition());
  //SmartDashboard.putNumber("Intake/Pivot/Motor Output", armMoveMotor.getAppliedOutput());
    updateValues();
  }

  public void updateValues() {
    double currentkP = tunablekP;
    double currentkD = tunablekD;
    double currentkG = tunablekG;
   
    tunablekP = SmartDashboard.getNumber("Intake/Pivot/kP", tunablekP);
    tunablekI = SmartDashboard.getNumber("Intake/Pivot/kI", tunablekI);
    tunablekD = SmartDashboard.getNumber("Intake/Pivot/kD", tunablekD);
    tunablekG = SmartDashboard.getNumber("Intake/Pivot/kG", tunablekG);

    if (currentkP != tunablekP
        || currentkD != tunablekD
        || currentkG != tunablekG) {
      System.out.println("UPDATING VALUES");
      armController.setPID(currentkP, 0.0, currentkD);
      armFeedForward.setKg(currentkG);

      // moveConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
      // moveConfig.smartCurrentLimit(60); // No clue on this guy
      // moveConfig.closedLoop.maxMotion
      //         .cruiseVelocity(MAX_Velocity)
      //         .maxAcceleration(MAX_Acceleration)
      //         .allowedProfileError(MAX_LoopError);
      // moveConfig.closedLoop
      //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //         .pid(tunablekP, tunablekI, tunablekD)
      //         .outputRange(kMinOutput, kMaxOutput); //Max Fraction of Output

      // armMoveMotor.configure(moveConfig, ResetMode.kResetSafeParameters,
      // PersistMode.kPersistParameters);

    }
  }
}
