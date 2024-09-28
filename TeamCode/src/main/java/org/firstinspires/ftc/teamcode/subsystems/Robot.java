package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot extends Config {
    public Elevator elevator;
    public Claw claw;

    public Robot(HardwareMap hardwareMap) {
        Servo clawServo = hardwareMap.get(Servo.class, Config.clawServo);
        this.claw = new Claw(clawServo);

        Servo leftServo = hardwareMap.get(Servo.class, Config.leftServo);
        Servo rightServo = hardwareMap.get(Servo.class, Config.rightServo);
        this.elevator = new Elevator(leftServo, rightServo);
    }
}
