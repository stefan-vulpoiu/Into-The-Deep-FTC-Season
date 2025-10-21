package org.firstinspires.ftc.teamcode.autonomous

import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand
import com.rowanmcalpin.nextftc.core.command.utility.SingleFunctionCommand
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil
import com.rowanmcalpin.nextftc.core.units.deg
import com.rowanmcalpin.nextftc.pedro.FollowPath
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.detection.AutoAimCommand
import org.firstinspires.ftc.teamcode.detection.LimelightDetection
import org.firstinspires.ftc.teamcode.detection.ROMParameters
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.util.AutonomousBase
import org.firstinspires.ftc.teamcode.util.UtilityCommands.enterCollect
import org.firstinspires.ftc.teamcode.util.commands.ConditionalCommand
import kotlin.math.abs
import kotlin.math.asin

@Autonomous(name = "Sample Auto", group = "Autonomous Sample")
class SampleAuto : AutonomousBase(IntakeSubsystem, OuttakeSubsystem) {
    private lateinit var detection: LimelightDetection

    private val startPose = Pose(7.8, 113.5, 0.0.deg.inRad)
    private val depositPose = Pose(16.0, 129.6, -40.0.deg.inRad)

    private val collectSpikePose1 = Pose(20.0, 126.6, -23.0.deg.inRad)
    private val collectSpikePose2 = Pose(19.55, 133.8, 2.5.deg.inRad)
    private val collectSpikePose3 = Pose(19.75, 134.7, 0.0.deg.inRad)

    private val collectSubmersiblePose = Pose(61.5, 101.25, -90.0.deg.inRad)
    private val depositSubmersiblePose = Pose(17.0, 132.5, -22.5.deg.inRad)

    private val parkPose = Pose(57.0, 100.5, -90.0.deg.inRad)

    private lateinit var depositPreloadPath: PathChain

    private lateinit var collectSpikePath1: PathChain
    private lateinit var collectSpikePath2: PathChain
    private lateinit var collectSpikePath3: PathChain

    private lateinit var depositSpikePath1: PathChain
    private lateinit var depositSpikePath2: PathChain
    private lateinit var depositSpikePath3: PathChain

    private lateinit var collectSubmersiblePath: PathChain
    private lateinit var collectSubmersibleFirstPath: PathChain

    private lateinit var depositSubmersiblePath: PathChain

    private lateinit var parkPath: PathChain

    private var failCount = 0

    private fun buildPaths() {
        depositPreloadPath = follower.pathBuilder()
            .addPath(
                BezierLine(
                    startPose,
                    depositPose
                )
            )
            .setLinearHeadingInterpolation(startPose.heading, depositPose.heading)
            .setZeroPowerAccelerationMultiplier(1.8)
            .setPathEndTValueConstraint(0.86)
            .build()

        collectSpikePath1 = follower.pathBuilder()
            .addPath(
                BezierLine(
                    depositPose,
                    collectSpikePose1
                )
            )
            .setLinearHeadingInterpolation(depositPose.heading, collectSpikePose1.heading)
            .build()

        depositSpikePath1 = follower.pathBuilder()
            .addPath(
                BezierLine(
                    collectSpikePose1,
                    depositPose
                )
            )
            .setLinearHeadingInterpolation(collectSpikePose1.heading, depositPose.heading)
            .build()

        collectSpikePath2 = follower.pathBuilder()
            .addPath(
                BezierLine(
                    depositPose,
                    collectSpikePose2
                )
            )
            .setLinearHeadingInterpolation(depositPose.heading, collectSpikePose2.heading)
            .build()

        depositSpikePath2 = follower.pathBuilder()
            .addPath(
                BezierLine(
                    collectSpikePose2,
                    depositPose
                )
            )
            .setLinearHeadingInterpolation(collectSpikePose2.heading, depositPose.heading)
            .build()

        collectSpikePath3 = follower.pathBuilder()
            .addPath(
                BezierLine(
                    depositPose,
                    collectSpikePose3
                )
            )
            .setLinearHeadingInterpolation(depositPose.heading, collectSpikePose3.heading)
            .build()

        depositSpikePath3 = follower.pathBuilder()
            .addPath(
                BezierLine(
                    collectSpikePose3,
                    depositPose
                )
            )
            .setLinearHeadingInterpolation(collectSpikePose3.heading, depositPose.heading)
            .build()

        collectSubmersibleFirstPath = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    depositPose,
                    Pose(collectSubmersiblePose.x, 122.3),
                    collectSubmersiblePose,
                )
            )
            .setLinearHeadingInterpolation(depositPose.heading, collectSubmersiblePose.heading)
            .setZeroPowerAccelerationMultiplier(3.8)
            .setPathEndTValueConstraint(0.82)
            .build()

        collectSubmersiblePath = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    depositSubmersiblePose,
                    Pose(collectSubmersiblePose.x, 130.5),
                    collectSubmersiblePose,
                )
            )
            .setLinearHeadingInterpolation(
                depositSubmersiblePose.heading,
                collectSubmersiblePose.heading
            )
            .setZeroPowerAccelerationMultiplier(3.0)
            .setPathEndTValueConstraint(0.78)
            .build()

        depositSubmersiblePath = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    collectSubmersiblePose,
                    Pose(52.6, 113.0),
                    depositSubmersiblePose,
                )
            )
            .setLinearHeadingInterpolation(
                collectSubmersiblePose.heading,
                depositSubmersiblePose.heading
            )
            .setPathEndTValueConstraint(0.81)
            .setZeroPowerAccelerationMultiplier(3.0)
            .build()

        parkPath = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    depositSubmersiblePose,
                    Pose(parkPose.x, 130.5),
                    parkPose,
                )
            )
            .setLinearHeadingInterpolation(
                depositSubmersiblePose.heading,
                parkPose.heading
            )
            .setZeroPowerAccelerationMultiplier(3.0)
            .setPathEndTValueConstraint(0.76)
            .build()
    }

    private val prepare
        get() = ParallelGroup(
            IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.BEHIND),
            IntakeSubsystem.setLiftPosition(0.2),
            IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
            IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.BEHIND),
            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.OPENED),
            IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN),

            OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.TRANSFER),
            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.CLOSED),
            OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),
            OuttakeSubsystem.setGearboxState(OuttakeSubsystem.GearboxState.NORMAL),
            OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.DOWN)
        )

    private val preload
        get() = SequentialGroup(
            FollowPath(depositPreloadPath, maxPower = 0.8, holdEnd = true)
                .and(
                    SequentialGroup(
                        Delay(.05),
                        OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET)
                            .endAfter(0.0),
                        OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.BASKET),
                        OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.OUT),

                        IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.LEFT),
                        IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.EXTENDING),
                        IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
                        IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.DOWN),
                    )
                ),

            WaitUntil { follower.velocityMagnitude <= 0.23 },

            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED).thenWait(0.075),
        )

    private val collectSpike1
        get() = SequentialGroup(
            FollowPath(collectSpikePath1, maxPower = 1.0, holdEnd = true)
                .and(
                    SequentialGroup(
                        OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.TRANSFER)
                            .thenWait(0.05),
                        OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),
                        OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.DOWN)
                            .endAfter(0.0),
                    )
                ),
            IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.OUT),

            Delay(0.1),

            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.DOWN),
            Delay(0.12),
            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),
            Delay(0.07),
            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.EXTENDING),
            IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.CENTER),

            FollowPath(depositSpikePath1, maxPower = 1.0, holdEnd = true)
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
                        Delay(0.075),

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
                    )
                ),

            Delay(.1),

            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED).thenWait(0.075),
        )

    private val collectSpike2
        get() = SequentialGroup(
            FollowPath(collectSpikePath2, maxPower = 1.0, holdEnd = true)
                .and(
                    SequentialGroup(
                        OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.TRANSFER)
                            .thenWait(0.05),
                        OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),
                        OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.DOWN)
                            .endAfter(0.0),
                    )
                ),
            IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.OUT),

            Delay(0.1),

            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.DOWN),
            Delay(0.12),
            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),
            Delay(0.07),
            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.EXTENDING),
            IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.CENTER),

            FollowPath(depositSpikePath2, maxPower = 1.0, holdEnd = true)
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
                        Delay(0.075),

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
                    )
                ),

            Delay(.15),

            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED).thenWait(0.075),
        )

    private val collectSpike3
        get() = SequentialGroup(
            FollowPath(collectSpikePath3, maxPower = 1.0, holdEnd = true)
                .and(
                    SequentialGroup(
                        OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.TRANSFER)
                            .thenWait(0.05),
                        OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),
                        OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.DOWN)
                            .endAfter(0.0),
                    )
                ),
            IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.OUT),

            Delay(0.1),
            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.DOWN),
            Delay(0.12),
            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),
            Delay(0.075),
            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK),

            FollowPath(depositSpikePath3, maxPower = 1.0, holdEnd = true)
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
                        Delay(0.075),

                        OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET)
                            .and(
                                SequentialGroup(
                                    IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.BEHIND),
                                    IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK),
                                    IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
                                    IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.BEHIND),

                                    Delay(.15),
                                    OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.BASKET),
                                    Delay(.05),
                                    OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.OUT),
                                    Delay(.05),
                                )
                            ),
                    )
                ),

            Delay(.15),

            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED).thenWait(0.075),
        )

    private val detectSubmersible: Command
        get() = SequentialGroup(
            InstantCommand {
                detection.runDetection()
                detection.resultExists = false
            },

            SingleFunctionCommand {
                detection.runDetection()
                detection.resultExists
            },

            Delay(0.05),

            SingleFunctionCommand {
                detection.runDetection()
                detection.resultExists
            },

            ConditionalCommand(
                { detection.worldCoordinates == null && failCount < 12 },
                {
                    SequentialGroup(
                        InstantCommand { failCount++ },

                        detectSubmersible
                    )
                },
                {
                    AutoAimCommand(
                        { detection.worldCoordinates?.x ?: 0.0 },
                        { detection.worldCoordinates?.y ?: 0.0 },
                        { detection.sampleYaw },
                    )
                }
            ),

            Delay(0.2),

            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.DOWN),
            Delay(.2),
            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),
            Delay(0.1),
            IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
            IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.UP),
            Delay(0.1),

            ConditionalCommand(
                { IntakeSubsystem.sampleSensor.getDistance(DistanceUnit.CM) > 1.8 },
                {
                    SequentialGroup(
                        IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.OPENED),
                        IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.BEHIND),
                        IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK),
                        IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
                        IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.BEHIND),
                        IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN),

                        Delay(0.2),

                        detectSubmersible
                    )
                }
            )
        )

    private val collectSubmersibleFirst
        get() = SequentialGroup(
            FollowPath(collectSubmersibleFirstPath, holdEnd = true)
                .and(
                    SequentialGroup(
                        OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.TRANSFER)
                            .thenWait(0.05),
                        OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),
                        OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.DOWN)
                            .endAfter(0.0),
                    )
                ),

            WaitUntil { follower.velocityMagnitude <= 0.1 },
            InstantCommand { failCount = 0 },

            detectSubmersible,

            FollowPath(depositSubmersiblePath, holdEnd = true)
                .and(
                    SequentialGroup(
                        IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.TRANSFER),

                        IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.CENTER),
                        IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.UP),

                        Delay(0.05),

                        IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),

                        IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN),
                        Delay(0.05),

                        OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.CLOSED),
                        Delay(0.075),
                        IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.OPENED),
                        Delay(0.075),

                        IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.TRANSFER)
                            .endAfter(0.0),
                        OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET)
                            .and(
                                SequentialGroup(
                                    IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.BEHIND),
                                    IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK),
                                    IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
                                    IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.BEHIND)
                                )
                            )
                    )
                ),

            OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.BASKET),
            Delay(.075),
            OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.OUT),
            Delay(.15),

            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED)
                .thenWait(0.075),
        )

    private val collectSubmersible
        get() = SequentialGroup(
            FollowPath(collectSubmersiblePath, holdEnd = true)
                .and(
                    SequentialGroup(
                        IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN),
                        OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.TRANSFER)
                            .thenWait(0.05),
                        OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),
                        OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.DOWN)
                            .endAfter(0.0),
                    )
                ),

            WaitUntil { follower.velocityMagnitude <= 0.1 },

            InstantCommand { failCount = 0 },

            detectSubmersible,

            FollowPath(depositSubmersiblePath, holdEnd = true)
                .and(
                    SequentialGroup(
                        IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.TRANSFER),

                        IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.CENTER),
                        IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.UP),

                        Delay(0.05),

                        IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),

                        IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN),
                        Delay(0.05),

                        OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.CLOSED),
                        Delay(0.075),
                        IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.OPENED),
                        Delay(0.075),

                        IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.TRANSFER)
                            .endAfter(0.0),
                        OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET)
                            .and(
                                SequentialGroup(
                                    IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.BEHIND),
                                    IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK),
                                    IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
                                    IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.BEHIND)
                                )
                            )
                    )
                ),

            OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.BASKET),
            Delay(.075),
            OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.OUT),
            Delay(.15),

            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED)
                .thenWait(0.075),
        )

    private val routine
        get() = ParallelGroup(
            SequentialGroup(
                preload,
                collectSpike1,
                collectSpike2,
                collectSpike3,
                collectSubmersibleFirst,
                collectSubmersible,

                FollowPath(parkPath, holdEnd = true, maxPower = 1.0)
                    .and(
                        SequentialGroup(
                            Delay(0.2),

                            enterCollect.endAfter(0.0),
                            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),

                            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.CLOSED),
                            OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.TRANSFER)
                                .thenWait(0.05),
                            OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),
                            OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.DOWN)
                                .endAfter(0.0),

                            IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN)
                                .endAfter(0.0),
                        )
                    ),
            ),
            SequentialGroup(
                Delay(29.0),
                FollowPath(parkPath, holdEnd = true, maxPower = 1.0)
                    .and(
                        SequentialGroup(
                            Delay(0.2),

                            enterCollect.endAfter(0.0),
                            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),

                            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.CLOSED),
                            OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.TRANSFER)
                                .thenWait(0.05),
                            OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),
                            OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.DOWN)
                                .endAfter(0.0),

                            IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN)
                                .endAfter(0.0),
                        )
                    ),
            )
        )

    override fun initialize() {
        follower = Follower(hardwareMap, FConstants::class.java, LConstants::class.java)
        follower.setStartingPose(startPose)

        detection = LimelightDetection(hardwareMap)

        buildPaths()

        prepare()
    }

    override fun onStartButtonPressed() {
        routine()
    }

    override fun update() {
        detection.runDetection()

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