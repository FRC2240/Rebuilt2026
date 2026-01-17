package frc.robot.subsystems;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

import frc.robot.Constants;

import static frc.robot.Constants.Climber.MOTOR_ID;


public class Climber extends SubsystemBase {
        TalonFX motor = new TalonFX(MOTOR_ID);
        TorqueCurrentFOC req = new TorqueCurrentFOC(0);

        Mechanism2d canvas = new Mechanism2d(27, 27);
        MechanismRoot2d climbRoot = canvas.getRoot("Climber", 2.25, 14.5);
        MechanismLigament2d climberSim = climbRoot.append(new MechanismLigament2d("Climber", 10, 90));

        public Climber(){
            TalonFXConfiguration conf = new TalonFXConfiguration();

            conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            
            motor.getConfigurator().apply(conf);
        }
    
    public void extend(Current current){
        motor.setControl(req.withOutput(current));
    }

    public Command extendCommand(Current current){
        return this.run(() -> {
            extend(current);
        });
    }

    public void extendSim(double length){
        climberSim.setLength(length);
    }

    public Command extendCommandSim(double length){
        return this.run(() -> {
            extendSim(length);
        });
    }
}

