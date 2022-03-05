package org.firstinspires.ftc.teamcode.a_opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.GlobalConfig
import org.firstinspires.ftc.teamcode.GlobalConfig.*
import org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline.TSEDetector
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive
import org.firstinspires.ftc.teamcode.b_hardware.Bot
import org.firstinspires.ftc.teamcode.b_hardware.subsystems.Outtake
import java.lang.Math.toRadians
import kotlin.math.PI

class AutoPaths(val opMode: LinearOpMode) {

    sealed class AutoPathElement(open val name: String) {
        class Path(override val name: String, val trajectory: Trajectory) : AutoPathElement(name)

        //AutoPathElement.Path(name, trajectory)
        class Action(override val name: String, val runner: () -> Unit) : AutoPathElement(name)
        //AutoPathElement.Action(name) {actions to take(include sleeps)}
    }

    val bot: Bot = Bot.getInstance()
    val drive: RRMecanumDrive = bot.roadRunner
    val paths: MutableList<AutoPathElement> = mutableListOf();

    val Double.toRadians get() = (toRadians(this))
    val Int.toRadians get() = (this.toDouble().toRadians)

    private fun Pose2d.reverse() = copy(heading = heading + PI)
    private var lastPosition: Pose2d = Pose2d()

    private var redAlliance = (GlobalConfig.alliance == GlobalConfig.Alliance.RED)
    private var depotSide = (GlobalConfig.side == GlobalConfig.Side.DEPOT)

    private var multiplier = if (redAlliance) 1 else -1

    private val intakeDelay = 1
    private val carouselDelay = 2500
    private val reverseDelay = 500
    private val bucketDelay = 800

    val turnRadians = -3 * PI / 4


    fun makePath(name: String, trajectory: Trajectory): AutoPathElement.Path {
        lastPosition = trajectory.end()
        return AutoPathElement.Path(name, trajectory)
        //Start of list of trajectories should not be lastPosition
    }

    private val intakeStart = AutoPathElement.Action("start intake") {
        if (redAlliance) bot.intake.runRight() else bot.intake.runLeft()
    }

    private val intaking = AutoPathElement.Action("stop intake") {
        bot.roadRunner.mode = RRMecanumDrive.Mode.IDLE
        bot.roadRunner.setWeightedDrivePower(Pose2d((if (redAlliance) -1 else 1) * 0.25, 0.0, 0.0))
        while (if (redAlliance) !bot.intake.wasIntakedRight() else !bot.intake.wasIntakedLeft() && !bot.outtake.isFreightIn()) {
        }
        bot.roadRunner.setWeightedDrivePower(Pose2d(0.0, 0.0, 0.0))
        while (!bot.outtake.isFreightIn()) {
        }
        bot.roadRunner.mode = RRMecanumDrive.Mode.FOLLOW_TRAJECTORY
        if (redAlliance) bot.outtake.closeRightFlap() else bot.outtake.closeLeftFlap()
        if (redAlliance) bot.intake.reverseRight() else bot.intake.reverseLeft()
        //Thread.sleep(500)
        if (redAlliance) if (Math.abs(bot.outtake.rightFlap.position - Outtake.FLAP_CLOSED) < 0.07) else bot.outtake.openRightFlap()
        else (if (Math.abs(bot.outtake.leftFlap.position - Outtake.FLAP_CLOSED) < 0.05) else bot.outtake.openLeftFlap())
    }

    private val intakeStop = AutoPathElement.Action("stop intake") {
        bot.intake.stop()
    }

    private val outtakeHigh = AutoPathElement.Action("Outtake High") {
        bot.outtake.goToTopGoal()
        bot.outtake.autoRun()
    }

    private val outtakeMid = AutoPathElement.Action("Outtake Mid") {
        bot.outtake.goToMidGoal()
        bot.outtake.autoRun()
    }

    private val outtakeLow = AutoPathElement.Action("Outtake Low") {
        bot.outtake.goToLowGoal()
        bot.outtake.autoRun()
    }

    private val outtakeEnd = AutoPathElement.Action("Outtake End") {
        Thread.sleep(bucketDelay.toLong())
        bot.outtake.flipBucket()
        Thread.sleep(bucketDelay.toLong())
        bot.outtake.unFlipBucket()
        bot.outtake.fullyRetract()
        bot.outtake.autoRun()
        if (redAlliance) bot.outtake.closeLeftFlap() else bot.outtake.closeRightFlap()
        if (redAlliance) bot.outtake.openRightFlap() else bot.outtake.openLeftFlap()
    }

    // Kotlin 1.3 does not support inline instantiation of SAM interfaces
    class MarkerCallbackImpl(val func: () -> Unit) : MarkerCallback {
        override fun onMarkerReached() = func()
    }

    private val runCarousel = AutoPathElement.Action("Run carousel motor") {
        if (redAlliance) bot.carousel.runRed() else bot.carousel.runBlue()
    }

    private val carouselWait = AutoPathElement.Action("Wait for Carousel") {
        Thread.sleep(carouselDelay.toLong())
        bot.carousel.stop()
    }

    private val prepare = AutoPathElement.Action("Prepare for teleOp") {
        if (redAlliance) bot.outtake.closeLeftFlap() else bot.outtake.closeRightFlap()
        if (redAlliance) bot.outtake.openRightFlap() else bot.outtake.openLeftFlap()
    }

    fun asVector2D(pos: Pose2d): Vector2d {
        return Vector2d(pos.x, pos.y)
    }

    fun initialPosition(): Pose2d {
        val initialY = (if (redAlliance) -37.0 else -37.0)
        return Pose2d(
            multiplier * 65.0,
            initialY + (if (depotSide) 0.0 else 48.0),
            3 * PI / 2 - if (redAlliance) 0.0 else PI
        )
    }

    private fun carouselPosition(): Pose2d {
        return Pose2d(
                multiplier * (if (redAlliance) 60.0 else 59.0),
                (if (redAlliance) -65.0 else -67.0),
                3 * PI / 2 - if (redAlliance) 0.0 else PI
        )
    }

    private fun initialOuttakePosition(): Pose2d {
        val blueOffset = 4.0;
        return Pose2d(
                multiplier * 42.0,
                if (depotSide) -25.0 else (-2.0 + if (!redAlliance) blueOffset else 0.0),
                (5 * (PI / 4)) - (if (redAlliance) 3 * PI / 2 else PI) - (if (depotSide) (if (redAlliance) PI / 2 else 3 * PI / 2) else 0.0)
        )
    }

    private fun intakePosition(n: Int): Pose2d {
        var offset = 3.0
        return Pose2d(
                multiplier * 74.0,
                42.0 + offset * (n - 1),
                3 * PI / 2 - if (redAlliance) 0.0 else PI
        )
    }

    private fun intermediateWaypoint(): Pose2d {
        return Pose2d(multiplier * 74.0, 8.0, 3 * PI / 2 - if (redAlliance) 0.0 else PI)
    }

    private fun outtakePosition(n: Int): Pose2d {
        val offset = 7.0
        return Pose2d(multiplier * 51.0, -4.0 + offset * (n - 1), 3 * PI / 2 - if (redAlliance) 0.0 else PI)
    }

    private fun parkingPosition(): Pose2d {
        val blueOffset = -2.0;
        return if (depotSide) Pose2d(
                multiplier * (37.0 + if (!redAlliance) blueOffset else 0.0),
                -66.0,
                3 * PI / 2 - if (redAlliance) 0.0 else PI
        ) else Pose2d(72.0, 65.0, 3 * PI / 2 - if (redAlliance) 0.0 else PI)
    }

    private fun outtakeTangents(): List<Double> {
        val startTangent = PI / 2
        val endTangent = PI
        return listOf(
                startTangent - if (redAlliance) 0.0 else PI,
                endTangent - if (redAlliance) 0.0 else PI
        );
    }

    private fun intakeTangents(initialIntake: Boolean): List<Double> {
        val startTangent = 2 * PI
        val endTangent = PI / 2
        return listOf(
                startTangent - if (redAlliance) 0.0 else (if (depotSide) 12 * PI / 8 else PI),
                endTangent - if (redAlliance) 0.0 else PI
        )
    }

    private fun parkingTangents(): List<Double> {
        val startTangent = if (depotSide) 3 * PI / 2 else 2 * PI
        val endTangent = if (depotSide) 3 * PI / 2 else PI / 2

        return listOf(
                startTangent - if (redAlliance) 0.0 else PI,
                endTangent - if (redAlliance) 0.0 else PI
        )
    }

    private fun intakeTrajectory(n: Int, initialIntake: Boolean): List<AutoPathElement.Path> {

        var startingPose = if (carousel && n == 1)
            carouselPosition()
        else if (outtakeCube && n == 1)
            initialOuttakePosition()
        else if (n == 1 && !outtakeCube && !carousel)
            initialPosition()
        else
            outtakePosition(n)

        return return listOf(AutoPathElement.Path(
                "Intake $n",
                bot.roadRunner.trajectoryBuilder(startingPose, intakeTangents(initialIntake)[0])
                    .addTemporalMarker(0.1, intakeStart.runner)
                    .splineToLinearHeading(intermediateWaypoint(), intakeTangents(initialIntake)[1])
                    .build()),
                AutoPathElement.Path(
                    "Intake $n",
                bot.roadRunner.trajectoryBuilder(intermediateWaypoint(), intakeTangents(false)[1])
                    .splineToConstantHeading(asVector2D(intakePosition(n)), intakeTangents(false)[1])
                    .build())
        )
    }

    private fun outtakeTrajectory(
            n: Int
    ): AutoPathElement.Path {
        return AutoPathElement.Path(
                "Outtake $n", bot.roadRunner.trajectoryBuilder(bot.roadRunner.poseEstimate)
                .lineToConstantHeading(asVector2D(intermediateWaypoint()))
                .splineToLinearHeading(outtakePosition(n), outtakeTangents()[1])
                .build()
        )
    }

    private fun depotParkingTrajectory(): AutoPathElement.Path {
        var parkingTrajectory = bot.roadRunner.trajectoryBuilder(
                if (cycles < 1 && !carousel && !outtakeCube) initialPosition() else if (!carousel && outtakeCube) initialOuttakePosition() else if (carousel) carouselPosition() else outtakePosition(cycles),
                parkingTangents()[0])
                .splineToLinearHeading(parkingPosition(), parkingTangents()[1])
                .build()

        return AutoPathElement.Path(
                "Parking",
                parkingTrajectory)
    }

    private fun warehouseParkingTrajectory(): List<AutoPathElement.Path> {
        return intakeTrajectory(cycles, false)
    }

    private fun initialOuttakeTrajectory(element: TSEDetector.PipelineResult): AutoPathElement.Path {
        return makePath(
                "Outtake Preloaded Cube Trajectory",
                bot.roadRunner.trajectoryBuilder(initialPosition())
                        .addTemporalMarker(0.1, if (element == TSEDetector.PipelineResult.LEFT) outtakeHigh.runner else if (element == TSEDetector.PipelineResult.MIDDLE) outtakeMid.runner else outtakeLow.runner)
                        .lineToSplineHeading(initialOuttakePosition())
                        .build()
        )
    }

    val carouselTrajectory = makePath(
            "Carousel Trajectory",
            bot.roadRunner.trajectoryBuilder(initialOuttakePosition())
                    .lineToSplineHeading(carouselPosition())
                    .addTemporalMarker(0.01, runCarousel.runner)
                    .build()
    )

    val turnTrajectory =
            AutoPathElement.Action("Turn Trajectory") { bot.roadRunner.turn(-turnRadians) }
    val unTurnTrajectory =
            AutoPathElement.Action("Turn Trajectory") { bot.roadRunner.turn(turnRadians) }

    private fun calcTrajectories(
            pipelineResult: TSEDetector.PipelineResult,
            outtakeCube: Boolean,
            carousel: Boolean,
            cycles: Int,
            parking: Boolean
    ): List<AutoPathElement> {

        if (outtakeCube) {
            paths.add(initialOuttakeTrajectory(pipelineResult))
            paths.add(outtakeEnd)
        }

        if (carousel) {
            paths.add(carouselTrajectory)
            if (!redAlliance)
                paths.add(turnTrajectory)
            paths.add(carouselWait)
            if (!redAlliance)
                paths.add(unTurnTrajectory)
        }

        for (n in IntRange(1, cycles)) {
            //paths.add(intakeStart)
            paths.add((if (n == 1) intakeTrajectory(n, true) else intakeTrajectory(n, false))[0])
            paths.add((if (n == 1) intakeTrajectory(n, true) else intakeTrajectory(n, false))[1])
            paths.add(intaking)
            paths.add(outtakeTrajectory(n))
            paths.add(intakeStop)
            paths.add(outtakeHigh)
            paths.add(outtakeEnd)
        }

        if (parking) {
            paths.add(if (depotSide) depotParkingTrajectory() else warehouseParkingTrajectory()[0])
            if (!depotSide)
                paths.add(warehouseParkingTrajectory()[1])
        }

        return paths

    }

    //todo make it use periodic instead of sliderun
    // features as functions, that return command groups
    fun getTrajectories(a: TSEDetector.PipelineResult): List<AutoPathElement> {
        return calcTrajectories(
                a,
                outtakeCube,
                carousel,
                cycles,
                parking
        )
    }
}