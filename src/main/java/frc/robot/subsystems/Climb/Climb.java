package frc.robot.subsystems.Climb;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DeviceIDs.ShooterConstants;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

    // Motors

    private final SparkMax climbMotor = new SparkMax(ShooterConstants.climbCanId, MotorType.kBrushless);

 

    public Climb() {
          SparkMaxConfig climbConfig = new SparkMaxConfig();

          climbConfig.idleMode(IdleMode.kBrake);

          climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public Command climbUp() {
        return this.runEnd(() -> climbMotor.setVoltage(4),  () -> climbMotor.setVoltage(0));
    }

    public Command climbDrop() {
        return this.runEnd(() -> climbMotor.setVoltage(-4), () -> climbMotor.setVoltage(0));
    }
   
}
