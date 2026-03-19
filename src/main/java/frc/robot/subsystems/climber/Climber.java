package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.CoastOut;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    TalonFX motor = new TalonFX(ClimberConstants.MOTOR_ID);
    TorqueCurrentFOC req = new TorqueCurrentFOC(0);
    CoastOut coast = new CoastOut();

    public Climber() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 0;
        conf.Slot0.kD = 0;

        conf.CurrentLimits.SupplyCurrentLimit = 50;
        conf.CurrentLimits.StatorCurrentLimit = 100;
        
        conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(conf);
    }

    public Command extendCommand() {
        // Runs after setting control to prevent the default (coast) commmand from
        // being called until desired
        return this.runOnce(() -> motor.setControl(req.withOutput(ClimberConstants.EXTEND_CURRENT))).andThen(run(() -> {
        }));
    }

    public Command coastCommand() {
        return this.runOnce(() -> motor.setControl(coast));
    }
}
