package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElasticDashboard extends SubsystemBase{
    double minVoltage = 13;

    @Override
    public void periodic() {

        SmartDashboard.putNumber("ElasticDashboard/Voltage", RobotController.getBatteryVoltage());

        SmartDashboard.putBoolean("ElasticDashboard/HubActive", Field.isHubActive());

        publishPeriodTime();

        if (RobotController.getBatteryVoltage() < minVoltage) {minVoltage = RobotController.getBatteryVoltage();}

        SmartDashboard.putNumber("ElasticDashboard/MinimumVoltage", minVoltage);
    }

    public void publishPeriodTime() {
        double time = DriverStation.getMatchTime();
        double value = time;

        if (time < 0) {
            time = 0.0;
        }

        SmartDashboard.putNumber("ElasticDashboard/Matchtime", time);

        if (time > 130){
            value = time - 130;
            SmartDashboard.putString("ElasticDashboard/Phase", "Transition");
        }
        else if(time > 105) {
            value = time - 105;
            SmartDashboard.putString("ElasticDashboard/Phase", "Shift 1");
        }
        else if(time > 80) {
            value = time - 80;
            SmartDashboard.putString("ElasticDashboard/Phase", "Shift 2");
        }
        else if(time > 55) {
            value = time - 55;
            SmartDashboard.putString("ElasticDashboard/Phase", "Shift 3");
        }
        else if(time > 30) {
            if (Field.isHubActive()) {
                value = time;
            }
            else {
                value = time - 30;
            }
            
            SmartDashboard.putString("ElasticDashboard/Phase", "Shift 4");
        }
        else {
            value = time;
            SmartDashboard.putString("ElasticDashboard/Phase", "Auto/End");
        }

        SmartDashboard.putNumber("ElasticDashboard/Shifttime", value);
    }
}