package frc.robot.autostep;

import frc.robot.Shooter;

//import javax.swing.Timer;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Indexer;
import frc.robot.Robot;

public class Shoot extends AutoStep {

    public Shooter shooter;
    public Timer shootTimer;
    public Robot robot;
    public boolean run;

    public Shoot(Shooter shooter, Indexer indexer, Robot robot, boolean run) {
        super();
        shootTimer = new Timer();
        this.robot = robot;
        this.run = run;
        this.shooter = shooter;
    }

    public void Begin() {

    }

    public void Update() {
        robot.runShooter = run;
        robot.swerveDrive.drive(0, 0, 0, true, true);

        if (shooter.UpToSpeed(200)) {
            shootTimer.reset();
            shootTimer.start();
            isDone = true;
        }
    }
}