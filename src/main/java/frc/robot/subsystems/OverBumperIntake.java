package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants.IntakeConstants;

public class OverBumperIntake extends SubsystemBase {
    private SparkMax upperIntakeMotor = new SparkMax(IntakeConstants.UpperIntakeMotorID, MotorType.kBrushless);
    private SparkMax lowerIntakeMotor = new SparkMax(IntakeConstants.LowerIntakeMotorID, MotorType.kBrushless);

    private SparkMax leftFoldingMotor = new SparkMax(30, MotorType.kBrushless);
    private SparkMax rightFoldingMotor = new SparkMax(29, MotorType.kBrushless);

    public OverBumperIntake() {
        final SparkMaxConfig upperMotorConfig = new SparkMaxConfig();
        final SparkMaxConfig lowerMotorConfig = new SparkMaxConfig();

        upperMotorConfig.inverted(true);

        lowerMotorConfig.follow(upperIntakeMotor, true);

        this.upperIntakeMotor.configure(upperMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        this.lowerIntakeMotor.configure(lowerMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        final SparkMaxConfig leftFoldingMotorConfig = new SparkMaxConfig();
        final SparkMaxConfig rightFoldingMotorConfig = new SparkMaxConfig();

        leftFoldingMotorConfig.follow(rightFoldingMotor, true);

        this.rightFoldingMotor.configure(rightFoldingMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        this.leftFoldingMotor.configure(leftFoldingMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public Command intake() {
        return this.run(() -> {
            upperIntakeMotor.set(0.5);
        }).finallyDo(() -> {
            upperIntakeMotor.stopMotor();
        });

    }

    public Command intakePos() {
        return this.run(() -> {
            rightFoldingMotor.set(0.5);
        }).finallyDo(() -> {
            rightFoldingMotor.stopMotor();
        });

    }

    public Command outTakePos() {
        return this.run(() -> {
            rightFoldingMotor.set(-0.5);
        }).finallyDo(() -> {
            rightFoldingMotor.stopMotor();
        });

    }

}
