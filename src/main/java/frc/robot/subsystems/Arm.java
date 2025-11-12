package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.SparkModel;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.ArmConstants;
import frc.robot.Constants.DriveConstants.Climber.ClimberPositionsC;
import frc.robot.Constants.DriveConstants.Elevator.FeedForwardValues;

public class Arm extends SubsystemBase {
    private SparkMax armMotor = new SparkMax(ArmConstants.ArmMotorID, MotorType.kBrushless);

    private SparkBase sparkBase = new SparkBase(31, MotorType.kBrushless, SparkModel.SparkMax);

    private SparkRelativeEncoder encoder = new SparkRelativeEncoder(sparkBase);
    private ArmFeedforward feedforward = new ArmFeedforward(FeedForwardValues.kS,
            FeedForwardValues.kG,
            FeedForwardValues.kV);

    private double ffOutput;

    private ProfiledPIDController pidController = new ProfiledPIDController(
            ArmConstants.PIDValuesC.p,
            ArmConstants.PIDValuesC.i,
            ArmConstants.PIDValuesC.d,
            ArmConstants.constraints);

    public Arm() {
        final SparkMaxConfig armMotorConfig = new SparkMaxConfig();

        this.armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public Command inTake() {
        return this.run(() -> {
            armMotor.set(0.5);
        }).finallyDo(() -> {
            armMotor.stopMotor();
        });

    }

    public Command highPosCommand() {
        return this.runOnce(this::setHighPos);
    }

    public Command lowPosCommand() {
        return this.runOnce(this::setLowPos);
    }

    public void setHighPos() {
        this.setGoal(ArmConstants.outtakePos);
    }

    public void setLowPos() {
        this.setGoal(ArmConstants.intakePos);
    }

    public void setSpeed(double speed) {
        this.armMotor.set(speed);
    }

    private void setGoal(double input) {
        this.pidController.setGoal(
                MathUtil.clamp(input, ArmConstants.low, ArmConstants.upper));
    }

    public void stopClimber() {
        this.armMotor.stopMotor();
    }

    public double getEncoder() {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getMeasurement() {
        return getEncoder() * 2 * Math.PI;
    }

    public Command stopCommand() {
        return this.runOnce(this::stopClimber);
    }

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        ffOutput = -feedforward.calculate(setpoint.position, setpoint.velocity);
        output = -output;
        armMotor.setVoltage(ffOutput + output);
    }

    public void periodic() {
        useOutput(pidController.calculate(getMeasurement()),
                pidController.getSetpoint());

    }
}
