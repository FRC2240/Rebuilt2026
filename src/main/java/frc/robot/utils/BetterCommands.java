package frc.robot.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class BetterCommands extends Command {
    /**
     * Runs the command only while the condition is true.
     */
    public static Command runWhen(Command command, BooleanSupplier conditionSupplier) {
        return Commands.repeatingSequence(
                Commands.waitUntil(conditionSupplier),
                command.until(() -> !conditionSupplier.getAsBoolean()));
    }

    /**
     * Repeatedley chooses between the true and false command every loop depending
     * on the value of the condition.
     */
    public static Command repeatedlyChoose(Command trueCommand, Command falseCommand, BooleanSupplier conditionSupplier) {
        return Commands.repeatingSequence(
                trueCommand.onlyIf(() -> conditionSupplier.getAsBoolean())
                        .until(() -> !conditionSupplier.getAsBoolean()),
                falseCommand.onlyIf(() -> !conditionSupplier.getAsBoolean())
                        .until(() -> conditionSupplier.getAsBoolean()));
    }
}