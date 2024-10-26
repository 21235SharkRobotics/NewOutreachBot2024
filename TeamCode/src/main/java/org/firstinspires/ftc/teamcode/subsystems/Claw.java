package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo clawServo;

    public Claw(Servo clawServo) {
        this.clawServo = clawServo;

    }

    public void openClaw(double openPosition) {
        clawServo.setPosition(openPosition);
    }

    public void closeClaw(double closedPosition) {
        clawServo.setPosition(closedPosition);
    }

    public double returnPosition() {
        return clawServo.getPosition();
    }
}
