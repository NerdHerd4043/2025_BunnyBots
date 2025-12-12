package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.IntakeConstants;

public class Intakefolding extends SubsystemBase {
    private SparkMax leftFoldingMotor = new SparkMax(IntakeConstants.leftFoldingMotorID, MotorType.kBrushless);
    private SparkMax rightFoldingMotor = new SparkMax(IntakeConstants.rightFoldingMotorID, MotorType.kBrushless);

    public Intakefolding() {
        final SparkMaxConfig leftFoldingMotorConfig = new SparkMaxConfig();
        final SparkMaxConfig rightFoldingMotorConfig = new SparkMaxConfig();

        rightFoldingMotorConfig.idleMode(IdleMode.kBrake);
        leftFoldingMotorConfig.idleMode(IdleMode.kBrake);

        // leftFoldingMotorConfig.follow(rightFoldingMotor, true);

        this.rightFoldingMotor.configure(rightFoldingMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        this.leftFoldingMotor.configure(leftFoldingMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public Command intakePos() {
        return this.runOnce(() -> {
            rightFoldingMotor.set(0.1);
        }).finallyDo(() -> {
            rightFoldingMotor.stopMotor();
        });

    }

    public Command outtakePos() {
        return this.runOnce(() -> {
            rightFoldingMotor.set(-0.1);
        }).finallyDo(() -> {
            rightFoldingMotor.stopMotor();

        });

    }

}
