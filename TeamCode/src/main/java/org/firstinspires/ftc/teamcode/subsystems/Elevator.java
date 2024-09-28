package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Array;

public class Elevator {
    private Servo leftServo;
    private Servo rightServo;

    public static double leftUpPos;
    public static double rightUpPos;
    public static double leftDownPos;
    public static double rightDownPos;


    public Elevator(Servo leftServo, Servo rightServo) {
        this.leftServo = leftServo;
        this.rightServo = rightServo;
    }

    public void upPosition() {
        leftServo.setPosition(leftUpPos);
        rightServo.setPosition(rightUpPos);
    }

    public void downPosition() {
        leftServo.setPosition(leftDownPos);
        rightServo.setPosition(rightDownPos);
    }

    public double[] returnPosition() {
        return new double[] {leftServo.getPosition(), rightServo.getPosition()};
    }
}
