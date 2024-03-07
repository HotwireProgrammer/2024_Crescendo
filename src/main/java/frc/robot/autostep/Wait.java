package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;
import frc.robot.swerve.DriveSubsystem;

public class Wait extends AutoStep {

    public Timer driveTimer;
    public float length;
    public DriveSubsystem swerve;

    public Wait(float length, DriveSubsystem swerve) {
        super();
        this.length = length;
        this.swerve = swerve;
        driveTimer = new Timer();
    }

    public void Begin() {
        driveTimer = new Timer();
        driveTimer.reset();
        driveTimer.start();
    }

    public void Update() {
        swerve.drive(0, 0, 0, true, true);
        if (driveTimer.get() > length) {
            isDone = true;
        }
    }
}