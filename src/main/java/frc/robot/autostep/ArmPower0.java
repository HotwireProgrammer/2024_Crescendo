package frc.robot.autostep;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;
import frc.robot.Arm;

public class ArmPower0 extends AutoStep {

    public Timer armTimer;
    public float time;
    public float speed;
    public Arm arm;
    public Joystick operator;
    private boolean notABoolean;

    public ArmPower0(Arm arm, boolean notABoolean) {
        super();
        this.arm = arm;
        this.notABoolean = notABoolean;
    }

    public void Begin() {
    }

    public void Update() {

        arm.powerBool = notABoolean;
        isDone = true;
    }
}