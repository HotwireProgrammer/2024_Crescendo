package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;

public class Print extends AutoStep {

    public Timer driveTimer;
    public float length;

    public DriveTrain driveTrain;
    public String string;

    public Print(String string) {
        super();
        this.string = string;
    }

    public void Begin() {

    }

    public void Update() {
        System.out.println(string);
        isDone = true;
    }
}