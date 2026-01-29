package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

public class Spindexer extends SubsystemBase {

    private TalonFX spindexer = new TalonFX(SpindexerConstants.SPINDEXER_MOTOR_ID);

    public Spindexer() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0;
        spindexer.getConfigurator().apply(config);
        setDefaultCommand(disableCommand());

    }

    public void enable() {
        spindexer.setControl(new VelocityTorqueCurrentFOC(SpindexerConstants.ENABLED));
    }

    public void disable() {
        spindexer.stopMotor();
    }

    public Command enableCommand() {
        return this.runOnce(() -> enable()).andThen(run(() -> {}));
    }

    public Command disableCommand() {
        return this.runOnce(() -> disable());
    }

    public Command reverseCommand() {
        return this
                .runOnce(() -> spindexer.setControl(new VelocityTorqueCurrentFOC(SpindexerConstants.ENABLED.unaryMinus())));
    }

}
