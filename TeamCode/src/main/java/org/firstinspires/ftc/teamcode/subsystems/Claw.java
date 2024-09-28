package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo clawServo;
    public static double openPosition = 0;
    public static double closedPosition = 0;


    public Claw(Servo clawServo) {
        this.clawServo = clawServo;
    }

    public void openClaw() {
        clawServo.setPosition(openPosition);
    }

    public void closeClaw() {
        clawServo.setPosition(closedPosition);
    }

    public double returnPosition() {
        return clawServo.getPosition();
    }
}
