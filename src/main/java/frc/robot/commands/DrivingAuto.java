package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class DrivingAuto extends Command {
    Drivebase drivebase;
    public double AutoTime;
    public double CurrentTime;

    public DrivingAuto() {
        // Use addRequirements() here to declare subsystem dependencies.

        this.addRequirements(this.drivebase);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        AutoTime = Timer.getTimestamp();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        CurrentTime = Timer.getTimestamp();

        if ((CurrentTime - AutoTime) > 5) {

            drivebase.defaultDrive(0, 1, 0, true);

        } else {

            drivebase.defaultDrive(0, 0, 0, true);

        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        drivebase.defaultDrive(0, 0, 0, true);
        return true;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }

}