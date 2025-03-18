package org.firstinspires.ftc.teamcode.autonomus;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

import java.lang.Math;
import java.util.Arrays;
import java.util.concurrent.Executors;

@Autonomous
public class Bafta extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot;
        robot = new Robot(hardwareMap, telemetry, Executors.newScheduledThreadPool(1));

        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));

        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        drive.pinpoint.resetPosAndIMU();

        AccelConstraint accFast=new ProfileAccelConstraint(-100, 140);
        AccelConstraint accSlow = new ProfileAccelConstraint(-45.0, 90.0);

        VelConstraint speedFast= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(140.0),
                new AngularVelConstraint(Math.PI / 2)

        ));

        VelConstraint speedSlow= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(45.0),
                new AngularVelConstraint(Math.PI / 2)

        ));
        TrajectoryActionBuilder trBasket = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(7.5, 43, Math.toRadians(-45.0)), Math.toRadians(-220.0));
        TrajectoryActionBuilder trBack = trBasket.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(6.5, 45, Math.toRadians(-45.0)), Math.toRadians(-220.0));
        TrajectoryActionBuilder trFront = trBack.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(8, 44, Math.toRadians(-45.0)), Math.toRadians(-220.0));
        TrajectoryActionBuilder trSample1 = trBack.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(13.0, 34.0, 0.0), Math.toRadians(-220.0));

        TrajectoryActionBuilder trBasket1 = trSample1.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(7.5, 43, Math.toRadians(-45.0)), Math.toRadians(-220.0));

        // Final movement: Park
        TrajectoryActionBuilder trPark = trBasket1.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(43.0, 21.0, Math.toRadians(0.0)), Math.toRadians(0.0));




















//
//        TrajectoryActionBuilder trBasket = drive.actionBuilder(beginPose)
//                .splineToLinearHeading(new Pose2d(7, 42.5, Math.toRadians(-45.0)), Math.toRadians(-220.0));
//        TrajectoryActionBuilder trBack = trBasket.endTrajectory().fresh()
//                .splineToLinearHeading(new Pose2d(5.9, 6, Math.toRadians(-45.0)), Math.toRadians(-220.0));
//        TrajectoryActionBuilder trFront = trBack.endTrajectory().fresh()
//                .splineToLinearHeading(new Pose2d(8, 44, Math.toRadians(-45.0)), Math.toRadians(-220.0));
//
//        // Second movement: trSample1 (starts where trBasket ended)
//
//        TrajectoryActionBuilder trSample1 = trFront.endTrajectory().fresh()
//                .splineToLinearHeading(new Pose2d(13.0, 34.0, 0.0), Math.toRadians(-220.0));
//
//        TrajectoryActionBuilder trBasket1 = trSample1.endTrajectory().fresh()
//                .splineToLinearHeading(new Pose2d(8, 42.5, Math.toRadians(-45.0)), Math.toRadians(-220.0));
//




        // blocuri care se executa la init
//        Actions.runBlocking(robot.initSliderIntake());

        waitForStart();

        if (isStopRequested()) return;

        Action acttrBasket = trBasket.build();
        Action acttrBack = trBack.build();
        Action acttrFront = trFront.build();
        Action acttrSample1 = trSample1.build();
        Action acttrBasket1 = trBasket1.build();


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                               acttrBasket,
                                robot.raiseSlider(1300)
                        ),
                        new SleepAction(0.2),
                        robot.raiseSlider(3000),
                        new SleepAction(0.1),
                        robot.dropSample(),
                        acttrBack,
                        new SleepAction(1),
                        robot.dropSample1(),
                        acttrFront,
                        robot.resetOuttake(),
                        robot.raiseSlider(1300),
                        new SleepAction(0.5),
                        new ParallelAction(
                                robot.raiseSlider(0),
                                acttrSample1
                        ),
                        robot.intakeSample(),
                        new ParallelAction(
                                acttrBasket1,
                                robot.raiseSlider(1300)
                        ),
                        robot.raiseSlider(3000),
                        new SleepAction(0.1),
                        robot.dropSample(),
                        acttrBack,
                        new SleepAction(1),
                        robot.dropSample1(),
                        acttrFront,
                        robot.resetOuttake(),
                        robot.raiseSlider(0),
                        new SleepAction(1.5)
//                        acttrBasket,
//                        acttrBack,
//                        acttrFront,
//                        acttrSample1

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

}
