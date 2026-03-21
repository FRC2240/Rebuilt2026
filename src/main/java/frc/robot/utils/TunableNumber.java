package frc.robot.utils;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TunableNumber extends SubsystemBase {
    private double value;
    private String path;
    private Set<Consumer<Double>> listeners = new HashSet<>();

    /**
     * Creates a tunable number in Smart Dashboard with a default value.
     */
    public TunableNumber(String path, double defaultValue) {
        this.value = defaultValue;
        this.path = path;
        SmartDashboard.putNumber(path, defaultValue);
    }

    /**
     * Gets the value of the tunable number
     */
    public double get() {
        return value;
    }

    /**
     * Adds a change listener to the tunable number. Whenever the number is changed
     * over NT, the listener will trigger.
     */
    public void addChangeListener(Consumer<Double> listener) {
        this.listeners.add(listener);
    }

    /**
     * Removes a change listener from the tunable number.
     */
    public void removeChangeListener(Consumer<Double> listener) {
        if (!this.listeners.contains(listener)) {
            System.out.println("WARNING: Change listener removed without being added first on path " + path);
            return;
        }

        this.listeners.remove(listener);
    }

    @Override
    public void periodic() {
        double oldValue = value;
        value = SmartDashboard.getNumber(path, value);
        if (value != oldValue) {
            for (Consumer<Double> listener : listeners) {
                listener.accept(value);
            }
        }
    }
}
