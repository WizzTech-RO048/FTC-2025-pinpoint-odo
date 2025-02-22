package org.firstinspires.ftc.teamcode.autonomus;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Robot.Controller;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

import java.lang.Math;
import java.util.concurrent.Executors;

@TeleOp(name="slidere jos")
public class slider extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(
                hardwareMap,
                telemetry,
                Executors.newScheduledThreadPool(1)
        );

        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d lastPose = new Pose2d(0, 0, Math.toRadians(0));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        drive.pinpoint.resetPosAndIMU();

        robot = new Robot(hardwareMap, telemetry, Executors.newScheduledThreadPool(1));



    }

    @Override
    public void loop() {
        robot.slider.raiseSlider(-3000, 0.7);
    }

//    public void stop() { robot.stopRobot(); }

}