package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants.ArmConstants;
import frc.robot.Constants.DriveConstants.IntakeConstants;

public class OverBumperIntake extends SubsystemBase {
    private SparkMax upperIntakeMotor = new SparkMax(IntakeConstants.UpperIntakeMotorID, MotorType.kBrushless);
    private SparkMax lowerIntakeMotor = new SparkMax(IntakeConstants.LowerIntakeMotorID, MotorType.kBrushless);

    private SparkMax leftFoldingMotor = new SparkMax(IntakeConstants.leftFoldingMotorID, MotorType.kBrushless);
    private SparkMax rightFoldingMotor = new SparkMax(IntakeConstants.rightFoldingMotorID, MotorType.kBrushless);

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

        rightFoldingMotorConfig.idleMode(IdleMode.kBrake);

        // leftFoldingMotorConfig.follow(rightFoldingMotor, true);

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

    public void periodic() {

    }
}
