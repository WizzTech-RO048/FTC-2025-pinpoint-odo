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
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        drive.pinpoint.resetPosAndIMU();

        robot = new Robot(hardwareMap, telemetry, Executors.newScheduledThreadPool(1));

        waitForStart();

        if (isStopRequested()) return;

        TrajectoryActionBuilder trBasket = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(7.5, 43.5, Math.toRadians(-45)), Math.toRadians(-140));
        TrajectoryActionBuilder trSample1= drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(7.5, 43.5, Math.toRadians(-45)), Math.toRadians(-140));
        TrajectoryActionBuilder trSample2= drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(7.5, 43.5, Math.toRadians(-45)), Math.toRadians(-140));
        TrajectoryActionBuilder trSample3= drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(7.5, 43.5, Math.toRadians(-45)), Math.toRadians(-140));

        TrajectoryActionBuilder trPark = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(7.5, 43.5, Math.toRadians(-45)), Math.toRadians(-140));

        Actions.runBlocking(
                // ---- Basket + Slider --------
                new ParallelAction(
                        trBasket.build(),
                        raiseSlider(1300)
                ),
                // ----- Drop Preload ------
                new SequentialAction(
                        raiseSlider(2700),
                        dropSample(),
                        resetOuttake(),
                        raiseSlider(1300)
                ),
                // -----  --->Sample I Intake Trajectory------
                new ParallelAction(
                        raiseSlider(0),
                        trSample1.build()
                ),



                // ------ Pickup Sample I----
                intakeSample(),



                // ---- Basket Sample I
                new ParallelAction(
                        trBasket.build(),
                        raiseSlider(1300)
                ),
                // ----- Drop Sample ------
                new SequentialAction(
                        raiseSlider(2700),
                        dropSample(),
                        resetOuttake(),
                        raiseSlider(1300)
                ),
                // ----- --->Sample II Intake Trajectory------
                new ParallelAction(
                        raiseSlider(0),
                        trSample2.build()
                ),



                // ------ Pickup Sample II -----
                intakeSample(),



                new ParallelAction(
                        trBasket.build(),
                        raiseSlider(1300)
                ),
                // ----- Drop Sample II  ------
                new SequentialAction(
                        raiseSlider(2700),
                        dropSample(),
                        resetOuttake(),
                        raiseSlider(1300)
                ),
                // ----- --->Sample III  Trajectory------
                new ParallelAction(
                        raiseSlider(0),
                        trSample3.build()
                ),



                ///
                ///

                trPark.build()// Execute claw action after movement
        );
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
            sleep(150);
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
            robot.gripper.intake_release_position_initial();
            sleep(75);
            robot.gripper.outtake_grab_position();
            return false;

        };
    }
    public Action resetOuttake() {
        return packet -> {

            return false;
        };
    }
    public Action dropSample() {
        return packet -> {

            return false;
        };
    }
}
