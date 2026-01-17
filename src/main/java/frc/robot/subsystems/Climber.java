package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Current;

import static frc.robot.Constants.Climber.MOTOR_ID;

public class Climber extends SubsystemBase{
   TalonFX motor = new TalonFX(MOTOR_ID);
    TorqueCurrentFOC req = new TorqueCurrentFOC(0);
    private final TalonFXConfiguration conf;

    public Climber(){
        motor = new TalonFX(MOTOR_ID);
        conf = new TalonFXConfiguration();

        conf.Slot0.kP = 0;
        conf.Slot0.kD = 0;
        conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        motor.getConfigurator().apply(conf);
    }

    public void extend(Current current){
        motor.setControl(req.withOutput(current));
    }

    public Command extendCommand(Current current){
        return this.run(
            () ->  {
                extend(current);
            }
        );
    }    
}
