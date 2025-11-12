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
import frc.robot.Constants.DriveConstants.Climber.ClimberPositionsC;
import frc.robot.Constants.DriveConstants.IntakeConstants;

public class OverBumperIntake extends SubsystemBase {
    private SparkMax uperIntakeMotor = new SparkMax(IntakeConstants.UperIntakeMotorID, MotorType.kBrushless);
    private SparkMax lowerIntakeMotor = new SparkMax(IntakeConstants.LowerIntakeMotorID, MotorType.kBrushless);

    private ProfiledPIDController pidController = new ProfiledPIDController(
            IntakeConstants.PIDValuesC.p,
            IntakeConstants.PIDValuesC.i,
            IntakeConstants.PIDValuesC.d,
            IntakeConstants.constraints);

    OverBumperIntake() {
        final SparkMaxConfig uperMotorConfig = new SparkMaxConfig();
        final SparkMaxConfig lowerMotorConfig = new SparkMaxConfig();

        lowerMotorConfig.follow(uperIntakeMotor, true);

        this.uperIntakeMotor.configure(uperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.lowerIntakeMotor.configure(lowerMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

    }

    public Command highPosCommand() {
        return this.runOnce(this::setHighPos);
    }

    public Command lowPosCommand() {
        return this.runOnce(this::setLowPos);
    }

    public void setHighPos() {
        this.setGoal(IntakeConstants.outtakePos);
    }

    public void setLowPos() {
        this.setGoal(IntakeConstants.intakePos);
    }

    public void setSpeed(double speed) {
        this.uperIntakeMotor.set(speed);
    }

    private void setGoal(double input) {
        this.pidController.setGoal(
                MathUtil.clamp(input, 0.0, ClimberPositionsC.upper));
    }

    public void stopClimber() {
        this.uperIntakeMotor.stopMotor();
    }

    public Command stopCommand() {
        return this.runOnce(this::stopClimber);
    }

}
