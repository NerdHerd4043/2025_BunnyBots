package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.ArmConstants;

public class Arm extends SubsystemBase {
    private SparkMax armMotor = new SparkMax(ArmConstants.ArmMotorID, MotorType.kBrushless);

    public Arm() {
        final SparkMaxConfig armMotorConfig = new SparkMaxConfig();

        this.armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command outtake() {
        return this.run(() -> {
            armMotor.set(0.2);
        }).finallyDo(() -> {
            armMotor.stopMotor();
        });

    }

    public Command loadPos() {
        return this.run(() -> {
            armMotor.set(-0.05);
        }).finallyDo(() -> {
            armMotor.stopMotor();
        });

    }
}
