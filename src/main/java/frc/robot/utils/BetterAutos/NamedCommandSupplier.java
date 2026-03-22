package frc.robot.utils.BetterAutos;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class NamedCommandSupplier {
    private String name;
    private Supplier<Command> command;

    public NamedCommandSupplier(String key, Supplier<Command> runnable) {
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
