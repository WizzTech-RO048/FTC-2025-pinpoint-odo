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
public class BaftaPARCARE extends LinearOpMode {
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
                .splineToLinearHeading(new Pose2d(1, -43, Math.toRadians(0.0)), Math.toRadians(-220.0));



















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


        Actions.runBlocking(
                new SequentialAction(
                        acttrBasket
                )

        );


    }

}
