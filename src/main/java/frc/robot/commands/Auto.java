package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class Auto extends Command {
    private static Drivebase drivebase = new Drivebase();

    public Auto() {

    }

    public static Command auto() {
        return run(() -> drivebase.defaultDrive(0, 1, 0, true));
    }
}
