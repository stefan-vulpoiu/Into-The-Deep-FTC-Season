package org.firstinspires.ftc.teamcode.autonomous

import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup
import com.rowanmcalpin.nextftc.core.command.utility.SingleFunctionCommand
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil
import com.rowanmcalpin.nextftc.pedro.FollowPath
import org.firstinspires.ftc.teamcode.detection.AutoAimCommand
import org.firstinspires.ftc.teamcode.detection.LimelightDetection
import org.firstinspires.ftc.teamcode.detection.ROMParameters
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.util.AutonomousBase
import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.sqrt

@Disabled
@Autonomous(name = "Sample TehnoZ Auto", group = "Autonomous Sample")
class SampleTehnoZAuto : AutonomousBase(IntakeSubsystem, OuttakeSubsystem) {
    private lateinit var detection: LimelightDetection

    private val startPose = Pose(134.0, 43.2, Math.toRadians(90.0))

    private val scorePreloadPose = Pose(127.0, 15.0, Math.toRadians(160.0))

    private val intakeFirstPose = Pose(121.5, 16.5, Math.toRadians(160.0))
    private val scoreFirstPose = Pose(126.5, 12.0, Math.toRadians(180.0))

    private val intakeSecondPose = Pose(122.5, 13.5, Math.toRadians(180.0))
    private val scoreSecondPose = Pose(126.0, 11.0, Math.toRadians(180.0))

    private val intakeThirdPose = Pose(121.0, 12.0, Math.toRadians(180.0))
    private val scoreThirdPose = Pose(126.0, 11.0, Math.toRadians(180.0))

    private val intakeSubPose1 = Pose(85.0, 52.0, Math.toRadians(90.0))
    private val intakeSubPose2 = Pose(81.5, 52.0, Math.toRadians(90.0))
    private val intakeSubPose3 = Pose(78.0, 52.0, Math.toRadians(90.0))

    private val scoreBasketPose = Pose(128.5, 13.0, Math.toRadians(160.0))
    private val controlPointScore = Pose(82.0, 17.0)

    private val parkPose = Pose(135.0, 113.0, Math.toRadians(180.0))

    private lateinit var scorePreload: PathChain

    private lateinit var intakeFirst: PathChain
    private lateinit var scoreFirst: PathChain

    private lateinit var intakeSecond: PathChain
    private lateinit var scoreSecond: PathChain

    private lateinit var intakeThird: PathChain
    private lateinit var scoreThird: PathChain

    private lateinit var intakeSub1: PathChain
    private lateinit var intakeSub2: PathChain
    private lateinit var intakeSub3: PathChain

    private lateinit var scoreSub: PathChain

    private lateinit var park: PathChain

    fun closeEnough(target: Pose, threshholdDistance: Double, threshholdHeading: Double): Boolean {
        var deltaX = follower.pose.x - target.x
        var deltaY = follower.pose.y - target.y
        var deltaAngle = abs(
            follower.pose
                .heading - target.heading
        )
        deltaAngle = Math.toDegrees(deltaAngle)
        var distance = sqrt(deltaX * deltaX + deltaY * deltaY)
        return (distance <= threshholdDistance && deltaAngle <= threshholdHeading)
    }

    private fun buildPaths() {
        scorePreload = follower.pathBuilder() //Preload ul
            .addPath(BezierLine(startPose, scorePreloadPose))
            .setLinearHeadingInterpolation(startPose.heading, scorePreloadPose.heading)
            .setZeroPowerAccelerationMultiplier(3.0)
            .build()
        intakeFirst = follower.pathBuilder()
            .addPath(BezierLine(scorePreloadPose, intakeFirstPose))
            .setLinearHeadingInterpolation(
                scorePreloadPose.heading,
                intakeFirstPose.heading
            )
            .setZeroPowerAccelerationMultiplier(1.5)
            .build()
        scoreFirst = follower.pathBuilder()
            .addPath(BezierLine(intakeFirstPose, scoreFirstPose))
            .setLinearHeadingInterpolation(
                intakeFirstPose.heading,
                scoreFirstPose.heading
            )
            .setZeroPowerAccelerationMultiplier(2.3)
            .build()
        intakeSecond = follower.pathBuilder()
            .addPath(BezierLine(scoreFirstPose, intakeSecondPose))
            .setLinearHeadingInterpolation(
                scoreFirstPose.heading,
                intakeSecondPose.heading
            )
            .setZeroPowerAccelerationMultiplier(1.5)
            .build()
        scoreSecond = follower.pathBuilder()
            .addPath(BezierLine(intakeSecondPose, scoreSecondPose))
            .setLinearHeadingInterpolation(
                intakeSecondPose.heading,
                scoreSecondPose.heading
            )
            .setZeroPowerAccelerationMultiplier(2.3)
            .build()
        intakeThird = follower.pathBuilder()
            .addPath(BezierLine(scoreSecondPose, intakeThirdPose))
            .setLinearHeadingInterpolation(
                scoreSecondPose.heading,
                intakeThirdPose.heading
            )
            .setZeroPowerAccelerationMultiplier(1.95)
            .build()
        scoreThird = follower.pathBuilder()
            .addPath(BezierLine(intakeThirdPose, scoreThirdPose))
            .setLinearHeadingInterpolation(
                intakeThirdPose.heading,
                scoreThirdPose.heading
            )
            .build()
        intakeSub1 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    scoreBasketPose,
                    controlPointScore,
                    intakeSubPose1
                )
            )
            .setLinearHeadingInterpolation(
                scoreBasketPose.heading,
                intakeSubPose1.heading
            )
            .build()
        intakeSub2 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    scoreBasketPose,
                    controlPointScore,
                    intakeSubPose2
                )
            )
            .setLinearHeadingInterpolation(
                scoreBasketPose.heading,
                intakeSubPose2.heading
            )
            .build()
        intakeSub3 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    scoreBasketPose,
                    controlPointScore,
                    intakeSubPose3
                )
            )
            .setLinearHeadingInterpolation(
                scoreBasketPose.heading,
                intakeSubPose3.heading
            )
            .build()
        scoreSub = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    intakeSubPose2,
                    controlPointScore,
                    scoreBasketPose
                )
            )
            .setLinearHeadingInterpolation(
                intakeSubPose2.heading,
                scoreBasketPose.heading
            )
            .build()
        park = follower.pathBuilder() //Park
            .addPath(BezierLine(scoreBasketPose, parkPose))
            .setLinearHeadingInterpolation(scoreBasketPose.heading, parkPose.heading)
            .build()
    }

    private val prepare
        get() = ParallelGroup(
            IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN),
            IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.UP),
            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.OPENED),
            IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.DOWN),
            IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.CENTER),

            OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.TRANSFER),
            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.CLOSED),
            OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),
            OuttakeSubsystem.setGearboxState(OuttakeSubsystem.GearboxState.NORMAL),
            OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.DOWN)
        )


    private val preload
        get() = SequentialGroup(
            FollowPath(scorePreload, holdEnd = true).and(
                SequentialGroup(
                    IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.LEFT),
                    IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.EXTENDING),
                    IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
                    IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.DOWN),

                    OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.BASKET),
                    OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.OUT),
                    OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET),
                )
            ),

            WaitUntil { closeEnough(scorePreloadPose, 1.0, 40.0) },

            Delay(0.1),

            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED),

            Delay(0.1),

            OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.TRANSFER),
            OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),
            OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.DOWN)
                .endAfter(0.0),
            IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.OUT),

            Delay(0.4)
        )

    private val first
        get() = SequentialGroup(
            FollowPath(intakeFirst, holdEnd = true),
            Delay(0.3),

            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.DOWN),
            Delay(0.15),
            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),
            Delay(.075),

            FollowPath(scoreFirst, holdEnd = true)
                .and(
                    IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.TRANSFER),

                    IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
                    IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.UP),
                    IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.CENTER),
                    IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.UP),
                    Delay(0.05),
                    IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),

                    IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN),
                    Delay(0.05),

                    OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.CLOSED),
                    Delay(0.075),
                    IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.OPENED),
                    Delay(0.2),

                    OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET)
                        .and(
                            SequentialGroup(
                                IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.RIGHT),
                                IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.EXTENDING),
                                IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.LEFT),
                                IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.DOWN),

                                Delay(.05),
                                OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.BASKET),
                                Delay(.05),
                                OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.OUT),
                                Delay(.05)
                            )
                        ),
                ),

            WaitUntil { closeEnough(scoreFirstPose, 1.0, 40.0) },

            Delay(0.1),

            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED),

            Delay(0.1),

            OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.TRANSFER),
            OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),
            OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.DOWN)
                .endAfter(0.0),
            IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.OUT),

            Delay(0.4)
        )

    private val second
        get() = SequentialGroup(
            FollowPath(intakeSecond, holdEnd = true),
            Delay(0.3),

            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.DOWN),
            Delay(0.15),
            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),
            Delay(.075),

            FollowPath(scoreSecond, holdEnd = true)
                .and(
                    IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.TRANSFER),

                    IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
                    IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.UP),
                    IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.CENTER),
                    IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.UP),
                    Delay(0.05),
                    IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),

                    IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN),
                    Delay(0.05),

                    OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.CLOSED),
                    Delay(0.075),
                    IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.OPENED),
                    Delay(0.2),

                    OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET)
                        .and(
                            SequentialGroup(
                                IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.LEFT),
                                IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.EXTENDING),
                                IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.RIGHT),
                                IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.DOWN),

                                Delay(.05),
                                OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.BASKET),
                                Delay(.05),
                                OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.OUT),
                                Delay(.05)
                            )
                        ),
                ),

            WaitUntil { closeEnough(scoreSecondPose, 1.0, 40.0) },

            Delay(0.1),

            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED),

            Delay(0.1),

            OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.TRANSFER),
            OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),
            OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.DOWN)
                .endAfter(0.0),
            IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.OUT),

            Delay(0.4)
        )

    private val third
        get() = SequentialGroup(
            FollowPath(intakeThird, holdEnd = true),
            Delay(0.3),

            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.DOWN),
            Delay(0.15),
            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),
            Delay(.075),

            FollowPath(scoreThird, holdEnd = true)
                .and(
                    IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.TRANSFER),

                    IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
                    IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.UP),
                    IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.CENTER),
                    IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.UP),
                    Delay(0.05),
                    IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),

                    IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN),
                    Delay(0.05),

                    OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.CLOSED),
                    Delay(0.075),
                    IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.OPENED),
                    Delay(0.2),

                    OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET)
                        .and(
                            SequentialGroup(
                                IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.BEHIND),
                                IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK),
                                IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
                                IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.BEHIND),

                                Delay(.05),
                                OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.BASKET),
                                Delay(.05),
                                OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.OUT),
                                Delay(.05)
                            )
                        ),
                ),

            WaitUntil { closeEnough(scoreThirdPose, 1.0, 40.0) },

            Delay(0.1),

            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED),

            Delay(0.1),

            OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.TRANSFER),
            OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),
            OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.DOWN)
                .endAfter(0.0),
            IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.OUT),

            Delay(0.4)
        )

    private val sub
        get() = SequentialGroup(
            WaitUntil { follower.velocityMagnitude <= 0.22 },

            SingleFunctionCommand {
                detection.runDetection()
                detection.resultExists
            },

            AutoAimCommand(
                { detection.worldCoordinates?.x ?: 0.0 },
                { detection.worldCoordinates?.y ?: 0.0 },
                { detection.sampleYaw },
            ),

            FollowPath(scoreSub, holdEnd = true),

            Delay(0.075),

            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.DOWN),
            Delay(.1),
            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),
            Delay(0.075),

            FollowPath(scoreSub, holdEnd = true)
                .and(
                    SequentialGroup(
                        IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.TRANSFER),

                        IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
                        IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.UP),
                        IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.CENTER),
                        IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.UP),

                        Delay(0.05),

                        IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),

                        IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN),
                        Delay(0.05),

                        OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.CLOSED),
                        Delay(0.075),
                        IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.OPENED),
                        Delay(0.2),

                        IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.OUT)
                            .endAfter(0.0),
                        OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET)
                            .and(
                                SequentialGroup(
                                    IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.BEHIND),
                                    IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK),
                                    IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
                                    IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.BEHIND),


                                    Delay(.05),
                                    OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.BASKET),
                                    Delay(.05),
                                    OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.OUT),
                                    Delay(.05)
                                )
                            )
                    )
                ),

            WaitUntil { closeEnough(scoreBasketPose, 1.0, 40.0) },

            Delay(0.1),

            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED),

            Delay(0.1),

            OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.TRANSFER),
            OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),
            OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.DOWN)
                .endAfter(0.0),
            IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN)
                .endAfter(0.0),

            Delay(0.4)
        )

    private val routine
        get() = SequentialGroup(
            preload,
            first,
            second,
            third,
            FollowPath(intakeSub1, holdEnd = true),
            sub,
            FollowPath(intakeSub2, holdEnd = true),
            sub,
            FollowPath(intakeSub3, holdEnd = true),
            sub,
            FollowPath(park, holdEnd = true)
        )

    override fun initialize() {
        detection = LimelightDetection(hardwareMap)

        follower = Follower(hardwareMap, FConstants::class.java, LConstants::class.java)
        follower.setStartingPose(startPose)

        buildPaths()

        prepare()
    }

    override fun onStartButtonPressed() {
        routine()
    }

    override fun update() {
        if (detection.resultExists) {
            telemetry.addData("aspectRatio", detection.aspectRatio)
            telemetry.addData("X", detection.worldCoordinates?.x)
            telemetry.addData("Y", detection.worldCoordinates?.y)
            telemetry.addData("Z", detection.worldCoordinates?.z)
            telemetry.addData("tx", detection.horizontalAngle)
            telemetry.addData("ty", detection.verticalAngle)
            telemetry.addData("Yaw", Math.toDegrees(0.0))
            telemetry.addData(
                "turretAngle",
                Math.toDegrees(
                    asin(
                        abs(
                            detection.worldCoordinates?.x ?: 0.0
                        ) / ROMParameters.armLength
                    )
                )
            )
            telemetry.addData(
                "turretPos",
                asin(
                    abs(
                        detection.worldCoordinates?.x ?: 0.0
                    ) / ROMParameters.armLength
                ) * 0.12 / Math.toRadians(
                    30.0
                )
            )
        } else telemetry.addLine("No result.")
    }
}