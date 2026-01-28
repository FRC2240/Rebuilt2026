package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

public class spindexer extends SubsystemBase {

    private TalonFX spindexer = new TalonFX(spindexerConstants.SPINDEXER_MOTOR_ID);

    public spindexer() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0;
        spindexer.getConfigurator().apply(config);

    }

    public void enable() {
        spindexer.setControl(new VelocityTorqueCurrentFOC(spindexerConstants.ENABLED));
    }

    public void disable() {
        spindexer.stopMotor();
    }

    public Command enableCommand() {
        return this.runOnce(() -> enable());
    }

    public Command disableCommand() {
        return this.runOnce(() -> disable());
    }

    public Command reverseCommand() {
        return this
                .runOnce(() -> spindexer.setControl(new VelocityTorqueCurrentFOC(spindexerConstants.ENABLED.unaryMinus())));
    }

}
