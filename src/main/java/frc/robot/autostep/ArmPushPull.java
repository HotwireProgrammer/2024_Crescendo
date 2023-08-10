package frc.robot.autostep;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;
import frc.robot.Arm;

public class ArmPushPull extends AutoStep {

    public Timer armTimer;
    public float time;
    public Arm arm;
    public boolean retract;

    public ArmPushPull(Arm arm, float time, boolean retract) {
        super();
        this.time = time;
        this.arm = arm;
        this.retract = retract;
        armTimer = new Timer();
    }

    public void Begin() {
        armTimer.reset();
        armTimer.start();
    }

    public void Update() {
        arm.Extend(retract);
        if (armTimer.get() > time) {
            isDone = true;
        }
    }
}