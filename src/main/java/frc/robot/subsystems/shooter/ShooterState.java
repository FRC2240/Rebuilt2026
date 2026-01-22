package frc.robot.subsystems.shooter;

public class ShooterState {

    private double speed;
    public boolean canHit = true;
    
    public ShooterState(double spd) {
        speed = spd;
    }
    
    public ShooterState(double spd, boolean isPossible) {
        speed = spd;
        canHit = isPossible;
    }

    public void add(ShooterState other) {
        speed += other.getSpeed();

    }

    public double getSpeed() {
        return speed;
    }
}
