package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.*
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity
import java.util.*


object MeepMeepTesting {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep: MeepMeep = MeepMeep(800)
        val beginPose = Pose2d(0.0, 0.0, Math.toRadians(0.0))

        val myBot: RoadRunnerBotEntity =
                DefaultBotBuilder(meepMeep)
                        .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
                        .build()

        val drive = myBot.drive

        // First movement: trBasket
        // First movement: trBasket


        val accFast: AccelConstraint = ProfileAccelConstraint(-100.0, 140.0)
        val accSlow: AccelConstraint = ProfileAccelConstraint(-45.0, 90.0)

        val speedFast: VelConstraint = MinVelConstraint(Arrays.asList(
                TranslationalVelConstraint(140.0),
                AngularVelConstraint(Math.PI / 2)

        ))

        val speedSlow: VelConstraint = MinVelConstraint(Arrays.asList(
                TranslationalVelConstraint(45.0),
                AngularVelConstraint(Math.PI / 2)

        ))
        // First movement: trBasket
        val trBasket = drive.actionBuilder(beginPose)
                .splineToLinearHeading(Pose2d(7.5, 43.0, Math.toRadians(-45.0)), Math.toRadians(-220.0))
        val trBack = trBasket.endTrajectory().fresh()
                .splineToLinearHeading(Pose2d(6.5, 45.0, Math.toRadians(-45.0)), Math.toRadians(-220.0))
        val trFront = trBack.endTrajectory().fresh()
                .splineToLinearHeading(Pose2d(8.0, 44.0, Math.toRadians(-45.0)), Math.toRadians(-220.0))
        val trSample1 = trBack.endTrajectory().fresh()
                .splineToLinearHeading(Pose2d(13.0, 34.0, 0.0), Math.toRadians(-220.0))

        val trBasket1 = trSample1.endTrajectory().fresh()
                .splineToLinearHeading(Pose2d(7.5, 43.0, Math.toRadians(-45.0)), Math.toRadians(-220.0))


        // Final movement: Park
        val trPark = trBasket1.endTrajectory().fresh()
                .splineToLinearHeading(Pose2d(43.0, 21.0, Math.toRadians(0.0)), Math.toRadians(0.0))


        // Execute all actions in sequence
        myBot.runAction(
                SequentialAction(

                        trBasket.build(),
                        trBack.build(),
                        trFront.build(),
                        trSample1.build()
        ))

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start()

    }
}
