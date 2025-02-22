package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity

object MeepMeepTesting {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep: MeepMeep = MeepMeep(800)
        var lastPose = Pose2d(0.0, 0.0, 0.0)  // Start position

        val myBot: RoadRunnerBotEntity =
                DefaultBotBuilder(meepMeep)
                        .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
                        .build()

        val drive = myBot.drive

        // First movement: trBasket
        // First movement: trBasket


        // First movement: trBasket
        val trBasket = drive.actionBuilder(lastPose)
                .splineToLinearHeading(Pose2d(7.5, 43.0, Math.toRadians(-45.0)), Math.toRadians(-220.0))
        lastPose = Pose2d(7.5, 43.0, Math.toRadians(-45.0)) // Update last position
        val trBack = drive.actionBuilder(lastPose)
                .splineToLinearHeading(Pose2d(6.5, 45.0, Math.toRadians(-45.0)), Math.toRadians(-220.0))
        lastPose = Pose2d(6.5, 45.0, Math.toRadians(-45.0)) // Update last position
        val trFront = drive.actionBuilder(lastPose)
                .splineToLinearHeading(Pose2d(8.0, 44.0, Math.toRadians(-45.0)), Math.toRadians(-220.0))
        lastPose = Pose2d(8.0, 44.0, Math.toRadians(-45.0)) // Update last position


        // Second movement: trSample1 (starts where trBasket ended)
        val trSample1 = drive.actionBuilder(lastPose)
                .splineToLinearHeading(Pose2d(13.0, 33.0, 0.0), Math.toRadians(-220.0))
        lastPose = Pose2d(13.0, 33.0, 0.0) // Update last position

        val trBasket1 = drive.actionBuilder(lastPose)
                .splineToLinearHeading(Pose2d(7.5, 43.0, Math.toRadians(-45.0)), Math.toRadians(-220.0))
        lastPose = Pose2d(7.5, 43.0, Math.toRadians(-45.0)) // Update last position


        // Final movement: Park
        lastPose = Pose2d(8.0, 44.0, Math.toRadians(-45.0)) // Update last position
        val trPark = drive.actionBuilder(lastPose)
                .splineToLinearHeading(Pose2d(40.0, 20.0, Math.toRadians(0.0)), Math.toRadians(0.0))
        lastPose = Pose2d(40.0, 20.0, Math.toRadians(0.0)) // Final last position update

        // Execute all actions in sequence
        myBot.runAction(
                SequentialAction(
                        trBasket.build(),
                        trBack.build(),
                        trFront.build(),
                        trSample1.build(),


                )
        )

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start()
    }
}
