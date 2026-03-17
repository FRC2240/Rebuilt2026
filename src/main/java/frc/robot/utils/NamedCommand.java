package frc.robot.utils;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class NamedCommand {
    private String name;
    private Supplier<Command> command;

    public NamedCommand(String key, Supplier<Command> runnable) {
        name = key;
        command = runnable;
    }

    public String GetName() {
        return name;
    }

    public Supplier<Command> GetCommand() {
        return command;
    }
}
