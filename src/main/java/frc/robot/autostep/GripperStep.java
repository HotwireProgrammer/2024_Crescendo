package frc.robot.autostep;

import frc.robot.Gripper;

public class GripperStep extends AutoStep {

    public Gripper gripper;
    public boolean doClose;

    public GripperStep(Gripper gripper, boolean doClose) {
        super();
        this.gripper = gripper;
        this.doClose = doClose;
    }

    public void Begin() {

    }

    public void Update() {
        if (doClose) {
            gripper.AutoClose();
        } else {
            gripper.AutoClear();
        }
        isDone=true;
    }
}
