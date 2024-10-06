package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Array;

@Config
public class Elevator {
    private Servo leftServo;
    private Servo rightServo;

    public Elevator(Servo leftServo, Servo rightServo) {
        this.leftServo = leftServo;
        this.rightServo = rightServo;

        this.leftServo.setDirection(Servo.Direction.REVERSE);
    }

    public void upPosition(double leftUpPos, double rightUpPos) {
        //leftServo.setPosition(leftUpPos);
        rightServo.setPosition(rightUpPos);
    }

    public void downPosition(double leftDownPos, double rightDownPos) {
        //leftServo.setPosition(leftDownPos);
        rightServo.setPosition(rightDownPos);
    }

    public double leftPosition() {
        return leftServo.getPosition();
    }

    public double rightPosition() {
        return rightServo.getPosition();
    }
}
