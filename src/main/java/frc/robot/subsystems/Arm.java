package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.ArmConstants;

public class Arm extends SubsystemBase {
    private SparkMax armMotor = new SparkMax(ArmConstants.ArmMotorID, MotorType.kBrushless);

    private CANcoder encoder = new CANcoder(ArmConstants.encoderID);
    private ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.FeedForwardValues.kS,
            ArmConstants.FeedForwardValues.kG,
            ArmConstants.FeedForwardValues.kV);

    private double ffOutput;

    // private SendableChooser<Double> PChooser = new SendableChooser<>();
    // private SendableChooser<Double> IChooser = new SendableChooser<>();
    // private SendableChooser<Double> DChooser = new SendableChooser<>();

    private ProfiledPIDController pidController = new ProfiledPIDController(
            ArmConstants.PIDValuesC.p,
            ArmConstants.PIDValuesC.i,
            ArmConstants.PIDValuesC.d,
            ArmConstants.constraints);

    public Arm() {
        final SparkMaxConfig armMotorConfig = new SparkMaxConfig();

        this.armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.pidController.setGoal(getEncoderRadians());

        // tune the arm
        SmartDashboard.putData(this.pidController);

    }

    public Command runMotorTest() {
        return this.run(() -> {
            armMotor.set(0.5);
        }).finallyDo(() -> {
            armMotor.stopMotor();
        });

    }

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        ffOutput = -feedforward.calculate(setpoint.position, setpoint.velocity);
        output = -output;
        armMotor.setVoltage(ffOutput + output);
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

    // public double getPChooser() {
    // return this.PChooser.getSelected();
    // }

    // public double getIChooser() {
    // return this.IChooser.getSelected();
    // }

    // public double getDChooser() {
    // return this.DChooser.getSelected();
    // }

    public void armUp() {
        setTarget(ArmConstants.outtakePos);
    }

    public void armDown() {
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

    public Command runArmDown() {
        return this.runOnce(() -> this.armDown());
    }

    public void periodic() {
        useOutput(pidController.calculate(getMeasurement()),
                pidController.getSetpoint());

        SmartDashboard.putNumber("ArmGoal", this.pidController.getGoal().position);
        SmartDashboard.putNumber("pos", this.getMeasurement());
        SmartDashboard.putNumber("encoder", this.getEncoder());

    }
}
