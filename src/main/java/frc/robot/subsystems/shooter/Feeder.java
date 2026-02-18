package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    private TalonFX motor = new TalonFX(ShooterConstants.FEEDER_MOTOR_ID);

    private VelocityTorqueCurrentFOC req = new VelocityTorqueCurrentFOC(0);

    public Feeder() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 5;
        conf.Slot0.kI = 0;
        conf.Slot0.kD = 0;
        // -80 30

        motor.getConfigurator().apply(conf);
        SmartDashboard.putNumber("Desired Feeder Velocity", ShooterConstants.FEED_VELOCITY.in(RotationsPerSecond));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder Velocity", motor.getVelocity().getValueAsDouble());
        ShooterConstants.FEED_VELOCITY = RotationsPerSecond.of(SmartDashboard.getNumber("Desired Feeder Velocity", 0));
    }
 
    public void setVelocity(AngularVelocity velocity) {
        motor.setControl(req.withVelocity(velocity));

    }

    public Command setVelocityCommand(Supplier<AngularVelocity> velocity) {
        return run(() -> setVelocity(velocity.get()));
    }

    public Command setEnabledCommand(Supplier<Boolean> enabledSupplier) {
        return run(() -> {
            boolean enabled = enabledSupplier.get();
            if (enabled) {
                setVelocity(ShooterConstants.FEED_VELOCITY);
            } else {
                setVelocity(RotationsPerSecond.of(0));
            }
        });
    }

    /**
     * Starts the shooter indexer motors
     */
    public Command enableCommand() {
        return runOnce(() -> setVelocity(ShooterConstants.FEED_VELOCITY)).andThen(run(() -> {
        }));
    }

    /**
     * Stops the shooter indexer motors.
     */
    public Command disableCommand() {
        return runOnce(() -> setVelocity(RotationsPerSecond.of(0))).andThen(run(() -> {
        }));
    }
}