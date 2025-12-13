package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.IntakeConstants;

public class Intake extends SubsystemBase {
    private SparkMax upperIntakeMotor = new SparkMax(IntakeConstants.UpperIntakeMotorID, MotorType.kBrushless);
    private SparkMax lowerIntakeMotor = new SparkMax(IntakeConstants.LowerIntakeMotorID, MotorType.kBrushless);

    public Intake() {
        final SparkMaxConfig upperMotorConfig = new SparkMaxConfig();
        final SparkMaxConfig lowerMotorConfig = new SparkMaxConfig();

        lowerMotorConfig.follow(upperIntakeMotor, true);

        this.upperIntakeMotor.configure(upperMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        this.lowerIntakeMotor.configure(lowerMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public Command intake() {
        return this.run(() -> {
            upperIntakeMotor.set(-0.5);
        }).finallyDo(() -> {
            upperIntakeMotor.stopMotor();
        });
    }

    public Command reverse() {
        return this.run(() -> {
            upperIntakeMotor.set(0.5);
        }).finallyDo(() -> {
            upperIntakeMotor.stopMotor();
        });
    }
}
