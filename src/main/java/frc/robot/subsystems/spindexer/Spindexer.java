package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

public class Spindexer extends SubsystemBase {
    private TalonFX motor = new TalonFX(SpindexerConstants.SPINDEXER_MOTOR_ID);

    public Spindexer() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 2;
        motor.getConfigurator().apply(config);
        SmartDashboard.putNumber("Desired Spindexer Velocity", SpindexerConstants.ENABLED_VELOCITY.in(RotationsPerSecond));

    }

    public void setVelocity(AngularVelocity velocity) {
        motor.setControl(new VelocityTorqueCurrentFOC(velocity));
    }
  
    @Override
    public void periodic() {
        SmartDashboard.putNumber("spindexer Velocity", motor.getVelocity().getValueAsDouble());
        SpindexerConstants.ENABLED_VELOCITY = RotationsPerSecond.of(SmartDashboard.getNumber("Desired Spindexer Velocity", 0));
    }

    public Command setVelocityCommand(AngularVelocity velocity) {
        return runOnce(() -> motor.setControl(new VelocityTorqueCurrentFOC(velocity)));
    }

    public Command setVelocityCommand(Supplier<AngularVelocity> velocity) {
        return run(() -> setVelocity(velocity.get()));
    }

    public Command enableCommand() {
        // Runs after setting control to prevent the default (disable) commmand from
        // being called until desired
        return setVelocityCommand(SpindexerConstants.ENABLED_VELOCITY).andThen(run(() -> {
        }));
    }

    public Command setEnabledCommand(Supplier<Boolean> enabledSupplier) {
        return run(() -> {
            boolean enabled = enabledSupplier.get();
            if (enabled) {
                setVelocity(SpindexerConstants.ENABLED_VELOCITY);
            } else {
                setVelocity(RotationsPerSecond.of(0));
            }
        });
    }

    public Command disableCommand() {
        return setVelocityCommand(RotationsPerSecond.of(0));
    }

    public Command reverseCommand() {
        return setVelocityCommand(SpindexerConstants.ENABLED_VELOCITY.unaryMinus()).andThen(run(() -> {
        }));
    }

}
