package frc.robot.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * This command type repeatedley checks the condition for which command to run. 
 * The Commands.either() command does not repeatedley check, even if decorated 
 * with .repeatedley(), thus this command exists.
 */
public class DynamicEither extends Command {
    private final Command trueCommand;
    private final Command falseCommand;
    private final BooleanSupplier condition;
    private Command currentlyRunning;

    public DynamicEither(Command trueCommand, Command falseCommand, BooleanSupplier condition) { 
        this.trueCommand = trueCommand;
        this.falseCommand = falseCommand;
        this.condition = condition;
        addRequirements(trueCommand.getRequirements());
    }

    @Override
    public void execute() {
        Command shouldRun = condition.getAsBoolean() ? trueCommand : falseCommand;
        
        if (shouldRun != currentlyRunning) {
            if (currentlyRunning != null) currentlyRunning.end(true);
            currentlyRunning = shouldRun;
            currentlyRunning.initialize();
        }
        currentlyRunning.execute();
    }

    @Override
    public void end(boolean interrupted) {
        if (currentlyRunning != null) currentlyRunning.end(interrupted);
    }
}