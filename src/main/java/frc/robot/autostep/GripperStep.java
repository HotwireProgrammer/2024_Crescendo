package frc.robot.autostep;

import frc.robot.Gripper;
import frc.robot.Robot;

public class GripperStep extends AutoStep {

    Robot robot;
    boolean state;

    public GripperStep(Robot robot, boolean state) {
        super();
        this.robot = robot;
        this.state = state;
    }

    public void Begin() {
        robot.autoGripperSet = state;
    }

    public void Update() {
        isDone=true;
    }
}
