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

    OverBumperIntake() {
        final SparkMaxConfig uperMotorConfig = new SparkMaxConfig();
        final SparkMaxConfig lowerMotorConfig = new SparkMaxConfig();

        lowerMotorConfig.follow(uperIntakeMotor, true);

        this.uperIntakeMotor.configure(uperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.lowerIntakeMotor.configure(lowerMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

    }

}
