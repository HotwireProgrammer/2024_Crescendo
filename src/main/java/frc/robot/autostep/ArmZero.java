package frc.robot.autostep;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;
import frc.robot.Arm;

public class ArmZero extends AutoStep {

    public Timer armTimer;
    public float time;
    public float speed;
    public Arm arm;
    public Joystick operator;

    public ArmZero(Arm arm) {
        super();
        this.arm = arm;
        armTimer = new Timer();
    }

    public void Begin() {
        armTimer.reset();
        armTimer.start();
    }

    public void Update() {
        arm.autoExtend = false;
        arm.RetractManual(0.4f);
        if (armTimer.get() > 0.5) {
            isDone = true;
            arm.ResetEncoder();
            arm.autoExtend = true;
        }
    }
}