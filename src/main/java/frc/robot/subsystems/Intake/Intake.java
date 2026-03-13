package frc.robot.subsystems.Intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;

import frc.robot.subsystems.DeviceIDs.IntakeConstants;
import frc.robot.subsystems.Intake.*;


public class Intake extends SubsystemBase {
    
    //Those Who Declare

    private final SparkFlex armMoveMotor = new SparkFlex(IntakeConstants.intakeMoveCanId, MotorType.kBrushless);
    private final SparkFlex spinIntakeMotor = new SparkFlex(IntakeConstants.intakeSpinCanId, MotorType.kBrushless);

    private final RelativeEncoder relativeMoveEncoder = armMoveMotor.getEncoder();

    private final SparkClosedLoopController armMovePID = armMoveMotor.getClosedLoopController();


    //Constants (untuned)
    private static double kP = 0.5;
    private static double kD = 0.002;
    private static double kG = 0.3;
    private static double kS = 0;
    
    private final ArmFeedforward moveFeedForward = new ArmFeedforward(kS, kG, 0);

    //Variables CUz  (need to be done, i dont rememb er them)
    private static final double arm_Position_Conversion_Factor = 1.0/20.0;
    private static final double arm_Velocity_Conversion_Factor = 4;
   
    //Offsets
    private static  double startPosition = 0;

    private double tunablekP = 0.05;
    private double tunablekD = 0.0;
    private double tunablekG = 0.1;
    private double tunableMaxAccel = 0.0;
    private double tunableMaxVelocity = 0.0;



    
    public Intake() {
        SparkFlexConfig moveConfig = new SparkFlexConfig();
        SparkFlexConfig spinConfig = new SparkFlexConfig();

        moveConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        moveConfig.smartCurrentLimit(60); // No clue on this guy

        //moveConfig.encoder.positionConversionFactor(arm_Position_Conversion_Factor);



        moveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(kP, 0, kD);
               // .outputRange(kMinOutput, kMaxOutput); //Max Fraction of Output
        

        armMoveMotor.configure(moveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        spinConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        spinConfig.smartCurrentLimit(60);
        spinConfig.encoder.velocityConversionFactor(arm_Velocity_Conversion_Factor);
        spinConfig.inverted(true);

        spinIntakeMotor.configure(spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        instantiateTunables();
        
    }

    private void instantiateTunables() {
        SmartDashboard.putNumber("Intake/Pivot/kP", kP);
        SmartDashboard.putNumber("Intake/Pivot/kD", kD);
        SmartDashboard.putNumber("Intake/Pivot/kG", kG);
        SmartDashboard.putNumber("Intake/Pivot/CurrentPosition", currentDegrees);
        SmartDashboard.putNumber("Intake/Pivot/Start Offset", startPosition);
        
    }
    
   
  

    public double getArmPositionDegrees() {
        return relativeMoveEncoder.getPosition();
    }

        double currentDegrees = getArmPositionDegrees();
        double currentArmRadians = Units.degreesToRadians(currentDegrees);
        
        double velocityRadPerSec = Units.degreesToRadians(relativeMoveEncoder.getVelocity());

        double ffVolts = moveFeedForward.calculate(currentArmRadians, velocityRadPerSec);

        
    // public Command setArmPosition(double targetDegrees) {
    //     return this.run(() -> 
    //     armMovePID.setSetpoint(targetDegrees, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, ffVolts));
    // }

    public FunctionalCommand setArmPosition(double angle) {
        return new FunctionalCommand(
            () -> {
                double setpoint = (angle*20.0)/(360);
                double gVal = moveFeedForward.calculate(getArmPositionDegrees() + frc.robot.subsystems.Intake.IntakeConstants.zeroOffset, 0.0);
                SmartDashboard.putNumber("Intake/Setpoint", angle);
                SmartDashboard.putNumber("Intake/Setpoint Converted", setpoint);
                armMovePID.setSetpoint(
                setpoint,
                SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, gVal);
            }, 
            () -> {
                double setpoint = (angle*20.0)/(360);
                double gVal = moveFeedForward.calculate(getArmPositionDegrees() + frc.robot.subsystems.Intake.IntakeConstants.zeroOffset, 0.0);
                armMovePID.setSetpoint(
                    setpoint,
                    SparkBase.ControlType.kPosition,
                    ClosedLoopSlot.kSlot0, gVal);
            }, 
            (interrupted) -> {
                armMoveMotor.set(0.0);
            },
            () -> false, this);
    }

    public Command intakeArmStop() {
      return this.runOnce(() -> armMoveMotor.set(0)); 
    }
    

    public Command intakeSpin(double voltage) {
        return this.runEnd(() -> spinIntakeMotor.setVoltage(voltage), () -> spinIntakeMotor.setVoltage(0)
        );
    }

    public FunctionalCommand runIntake(double voltage) {
        return new FunctionalCommand(
            () -> {
                spinIntakeMotor.setVoltage(voltage);
            }, () -> {}, 
            (interrupted) -> {
                spinIntakeMotor.setVoltage(0);
            }, 
            () -> false);
    }


    @Override
    public void periodic() {
        updateValues();
        SmartDashboard.putNumber("Intake/Pivot/CurrentPosition", armMoveMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake/Pivot/Raw Encoder", relativeMoveEncoder.getPosition());
        SmartDashboard.putNumber("Intake/Pivot/Output", armMoveMotor.getAppliedOutput());
    }

    public void updateValues() {
        double currentkP = tunablekP;
        double currentkD = tunablekD;
        double currentkG = tunablekG;
        double currentMaxAccel = tunableMaxAccel;
        double currentMaxVelocity = tunableMaxVelocity;
        double currentStartPosition = startPosition;
        double currentPosition = getArmPositionDegrees();

        tunablekP = SmartDashboard.getNumber("Intake/Pivot/kP", tunablekP);
        tunablekD = SmartDashboard.getNumber("Intake/Pivot/kD", tunablekD);
        tunablekG = SmartDashboard.getNumber("Intake/Pivot/kG", tunablekG);
        startPosition = SmartDashboard.getNumber("Intake/Pivot/Start Offset", startPosition);
        

        
        SparkFlexConfig moveConfig = new SparkFlexConfig();

        if (currentkP != tunablekP || currentkD != tunablekD || currentkG != tunablekG || 
            currentMaxAccel != tunableMaxAccel || currentMaxVelocity != tunableMaxVelocity || currentStartPosition != startPosition || currentPosition != currentDegrees) {
                System.out.println("UPDATING VALUES");
                moveConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
                moveConfig.smartCurrentLimit(60); // No clue on this guy
                //moveConfig.encoder.positionConversionFactor(arm_Position_Conversion_Factor);
                moveConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(tunablekP, 0, tunablekD);
                
                armMoveMotor.configure(moveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                
                currentStartPosition = startPosition;
                currentPosition = currentDegrees;
            }
        
    }



}
