package frc.robot.autostep;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;
import frc.robot.Arm;

public class ArmMove extends AutoStep {

    public Timer armTimer;
    public float time;
    public float speed;
    public Arm arm;
    public Joystick operator;

    public ArmMove(Arm arm, float time, float speed, Joystick operator) {
        super();
        this.time = time;
        this.speed = speed;
        this.arm = arm;
        this.operator = operator;
        armTimer = new Timer();
    }

    public void Begin() {
        armTimer.reset();
        armTimer.start();
    }

    public void Update() {
        arm.Update(speed, operator);
        if (armTimer.get() > time) {
            isDone = true;
            arm.Update(0, operator);
        }
    }
}