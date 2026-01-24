package frc.robot.subsystems.candle;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import com.ctre.phoenix6.controls.SolidColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Candle extends SubsystemBase {

    private final CANdle candle = new CANdle(CandleConstants.CANDLE_ID);
    private int count = 0;

    private boolean colorOn = true;
    private final Supplier<Boolean> shooting;
    private final Supplier<Boolean> canHit;

    private final SolidColor color = new SolidColor(0, CandleConstants.LED_NUM - 1);

    public Candle(Supplier<Boolean> shooting, Supplier<Boolean> canHit) { 
        this.shooting = shooting;
        this.canHit = canHit;
        
        LEDConfigs conf = new LEDConfigs();
        conf.StripType = StripTypeValue.RGBW;
        conf.BrightnessScalar = CandleConstants.BRIGTHNESS;
        candle.getConfigurator().apply(conf);

        color.Color = new RGBWColor(0, 0, 0, 0);
        candle.setControl(color);    
    }

    @Override
    public void periodic() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (DriverStation.isAutonomous()) {
            color.Color = new RGBWColor(255, 0, 255, 0);
        } else if (shooting.get()) {
            color.Color = new RGBWColor(255, 170, 0, 0);
        } else if(canHit.get()){
            color.Color = new RGBWColor(0, 255, 0, 0);
        } else {
            if (alliance.isEmpty()) {
                color.Color = new RGBWColor(255, 255, 255, 255);
            } else if (alliance.get().equals(Alliance.Red)) {
                color.Color = new RGBWColor(255, 0, 0, 0);
            } else {
                color.Color = new RGBWColor(0, 0, 255, 0);
            }

            if (DriverStation.isDisabled()) {
                count++;
            } else {
                // Ensures the colorOn is true when enabled
                colorOn = true;
            }
            // blinking every second if disabled
            if (count > 20) {
                count = 0;
                colorOn = !colorOn;
            }

            if (!colorOn) {
                color.Color = new RGBWColor(0, 0, 0, 0);
            }
        }

        candle.setControl(color);
    }

}
