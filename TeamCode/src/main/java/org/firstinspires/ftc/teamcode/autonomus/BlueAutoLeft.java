package org.firstinspires.ftc.teamcode.autonomus;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

import java.lang.Math;
import java.util.concurrent.Executors;

import static android.os.SystemClock.sleep;

@Autonomous
public class BlueAutoLeft extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d lastPose = new Pose2d(0, 0, Math.toRadians(0));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        drive.pinpoint.resetPosAndIMU();

        robot = new Robot(hardwareMap, telemetry, Executors.newScheduledThreadPool(1));

        waitForStart();

        if (isStopRequested()) return;


        // First movement: trBasket
        TrajectoryActionBuilder trBasket = drive.actionBuilder(lastPose)
                .splineToLinearHeading(new Pose2d(7.5, 43, Math.toRadians(-45.0)), Math.toRadians(-220.0));
        lastPose = new Pose2d(7.5, 43, Math.toRadians(-45.0)); // Update last position
        TrajectoryActionBuilder trBack = drive.actionBuilder(lastPose)
                .splineToLinearHeading(new Pose2d(6.5, 45, Math.toRadians(-45.0)), Math.toRadians(-220.0));
        lastPose = new Pose2d(6.2, 45.3, Math.toRadians(-45.0)); // Update last position
        TrajectoryActionBuilder trFront = drive.actionBuilder(lastPose)
                .splineToLinearHeading(new Pose2d(8, 44, Math.toRadians(-45.0)), Math.toRadians(-220.0));
        lastPose = new Pose2d(8, 44, Math.toRadians(-45.0)); // Update last position



        // Second movement: trSample1 (starts where trBasket ended)






        TrajectoryActionBuilder trSample1 = drive.actionBuilder(lastPose)
                .splineToLinearHeading(new Pose2d(13.0, 34.0, 0.0), Math.toRadians(-220.0));
        lastPose = new Pose2d(13.0, 34.0, 0.0); // Update last position

        TrajectoryActionBuilder trBasket1 = drive.actionBuilder(lastPose)
                .splineToLinearHeading(new Pose2d(7.5, 43, Math.toRadians(-45.0)), Math.toRadians(-220.0));
        lastPose = new Pose2d(7.5, 43, Math.toRadians(-45.0)); // Update last position

        // Final movement: Park
        lastPose = new Pose2d(8, 44, Math.toRadians(-45.0)); // Update last position
        TrajectoryActionBuilder trPark = drive.actionBuilder(lastPose)
                .splineToLinearHeading(new Pose2d(43.0, 21.0, Math.toRadians(0.0)), Math.toRadians(0.0));
        lastPose = new Pose2d(20.0, 35.0, Math.toRadians(0.0)); // Final last position update

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                trBasket.build(),
                                raiseSlider(1300)
                        ),
                        raiseSlider(2850),
                        Sleep(100),
                        dropSample(),
                        trBack.build(),
                        Sleep(1000),
                        dropSample1(),
                        trFront.build(),
                        resetOuttake(),
                        raiseSlider(1300),
                        Sleep(500),
                        new ParallelAction(
                                raiseSlider(0),
                                trSample1.build()
                        ),
                        intakeSample(),
                        new ParallelAction(
                                trBasket1.build(),
                                raiseSlider(1300)
                        ),
                        raiseSlider(2850),
                        Sleep(100),
                        dropSample(),
                        trBack.build(),
                        Sleep(1000),
                        dropSample1(),
                        trFront.build(),
                        resetOuttake(),
                        raiseSlider(0),
                        trPark.build()


//                        intakeSample(),
//                        new ParallelAction(
//                                trBasket.build(),
//                                raiseSlider(1300)
//                        ),
//                        new SequentialAction(
//                                raiseSlider(2700),
//                                dropSample(),
//                                resetOuttake(),
//                                raiseSlider(1300)
//                        ),
//                        new ParallelAction(
//                                raiseSlider(0),
//                                trSample3.build()
//                        ),
//                        trPark.build()
                )
        );


    }

    public Action Sleep(int value) {
        return packet -> {
            sleep(value);
            return false;
        };
    }

    public Action raiseSlider(int value) {
        return packet -> {
            robot.slider.raiseSlider(value, 1);
            return false;
        };
    }

    public Action intakeSample() {
        return packet -> {
            robot.gripper.outtake_release_position();
            robot.gripper.intake_grab_position();
            sleep(250);
            robot.horizontalSlider.setExtendedPosition();
            robot.gripper.pass_object_pickup_position();
            sleep(170);
            robot.gripper.intake_release_position();


            sleep(1000);


            robot.gripper.intake_grab_position();
            sleep(400);
            robot.gripper.pass_object_release_position();
            robot.gripper.intake_grab_position();
            sleep(700);
            robot.horizontalSlider.setStationaryPosition();
            sleep(1200);
            robot.gripper.intake_release_position();
            sleep(100);
            robot.gripper.outtake_grab_position();
            sleep(250);
            return false;

        };
    }

    public Action resetOuttake() {
        return packet -> {
            robot.gripper.outtake_grab_position();
            sleep(1000);
            robot.gripper.score_object_release_position();
            sleep(500);
            return false;
        };
    }


    public Action dropSample() {
        return packet -> {

            sleep(1000);
            robot.gripper.basket();
            sleep(1500);

            return false;
        };
    }
    public Action dropSample1() {
        return packet -> {
            robot.gripper.outtake_release_position();
            sleep(1000);
            return false;
        };
    }
}
