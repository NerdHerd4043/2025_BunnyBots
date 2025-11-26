package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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

    private CANcoder encoder = new CANcoder(IntakeConstants.encoderID);
    private ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.FeedForwardValues.kS,
            ArmConstants.FeedForwardValues.kG,
            ArmConstants.FeedForwardValues.kV);

    private double ffOutput;

    private ProfiledPIDController pidController = new ProfiledPIDController(
            IntakeConstants.PIDValuesC.p,
            IntakeConstants.PIDValuesC.i,
            IntakeConstants.PIDValuesC.d,
            IntakeConstants.constraints);

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

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        ffOutput = -feedforward.calculate(setpoint.position, setpoint.velocity);
        output = -output;
        rightFoldingMotor.setVoltage(ffOutput + output);
    }

    public void setTarget(double target) {
        // ArmPositions.upper is lower than ArmPositions.lower
        this.pidController.setGoal(MathUtil.clamp(target, ArmConstants.outtakePos,
                ArmConstants.intakePos));
    }

    public void setTargetRotations(double target) {
        setTarget(target * 2 * Math.PI);
    }

    public void adjustTarget(double delta) {
        if (Math.abs(delta) > 0.01) {
        }

        setTarget(this.pidController.getGoal().position + delta);
    }

    public void restPos() {
        setTarget(ArmConstants.outtakePos);
    }

    public void intakePos() {
        setTarget(ArmConstants.intakePos);
    }

    public double getEncoder() {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getEncoderRadians() {
        return getEncoder() * 2 * Math.PI;
    }

    public double getMeasurement() {
        // Return the process variable measurement here
        return getEncoderRadians();
    }

    public void periodic() {
        useOutput(pidController.calculate(getMeasurement()),
                pidController.getSetpoint());

        // SmartDashboard.putNumber("ArmGoal", this.pidController.getGoal().position);
        // SmartDashboard.putNumber("pos", this.getMeasurement());
        // SmartDashboard.putNumber("encoder", this.getEncoder());

    }

}
