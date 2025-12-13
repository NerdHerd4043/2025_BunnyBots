package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class Auto extends Command {

    public Auto() {
        Drivebase drivebase = new Drivebase();

        // drivebase = subsystems;
        // addRequirements(drivebase);
        drivebase.defaultDrive(0, 1, 0, true);

    }

    // @Override
    // public void initialize() {
    // drivebase.defaultDrive(0, 1, 0, true);
    // }

    // @Override
    // public boolean isFinished() {
    // return true;
    // }

}
