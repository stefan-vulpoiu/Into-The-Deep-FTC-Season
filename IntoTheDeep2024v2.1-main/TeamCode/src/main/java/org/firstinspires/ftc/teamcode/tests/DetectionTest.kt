package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup
import com.rowanmcalpin.nextftc.core.command.utility.NullCommand
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay
import org.firstinspires.ftc.teamcode.detection.AutoAimCommand
import org.firstinspires.ftc.teamcode.detection.LimelightDetection
import org.firstinspires.ftc.teamcode.detection.ROMParameters
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.util.TeleOpBase
import kotlin.math.abs
import kotlin.math.asin

@TeleOp(name = "Detection Test", group = "Tests")
class DetectionTest : TeleOpBase(DriveSubsystem, IntakeSubsystem) {
    private lateinit var detection: LimelightDetection

    private val reset
        get() = ParallelGroup(
            IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.BEHIND),
            IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.BEHIND),
            IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.OPENED),
            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.EXTENDING),
            IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN)
        )

    override fun initialize() {
        detection = LimelightDetection(hardwareMap)

        driver1.cross.pressedCommand = {
            if (detection.resultExists)
                SequentialGroup(
                    AutoAimCommand(
                        { detection.worldCoordinates?.x ?: 0.0 },
                        { detection.worldCoordinates?.y ?: 0.0 },
                        { detection.sampleYaw }
                    ),
                    Delay(.3),
                    IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.DOWN),
                    Delay(.2),
                    IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),
                    Delay(.12),
                    IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK)
                )
            else NullCommand()
        }

        driver1.circle.pressedCommand = { reset }
    }

    override fun onStartButtonPressed() {
        reset()
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