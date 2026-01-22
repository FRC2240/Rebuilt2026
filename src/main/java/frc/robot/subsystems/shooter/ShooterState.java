package frc.robot.subsystems.shooter;

public class ShooterState {

    double speedLeft;
    double speedRight;
    public boolean canHit = true;
    
    public ShooterState(double spdL, double spdR) {
        speedLeft = spdL;
        speedRight = spdR;
    }
    
    public ShooterState(double spdL, double spdR, boolean isPossible) {
        speedLeft = spdL;
        speedRight = spdR;
        canHit = isPossible;
    }

    public void add(ShooterState other) {
        speedLeft += other.getLeftSpeed();
        speedRight += other.getRightSpeed();

    }

    public double getLeftSpeed() {
        return speedLeft;
    }

    public double getRightSpeed() {
        return speedRight;
    }
}
