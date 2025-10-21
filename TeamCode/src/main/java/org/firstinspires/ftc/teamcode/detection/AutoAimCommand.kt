package org.firstinspires.ftc.teamcode.detection

import com.qualcomm.robotcore.util.ElapsedTime
import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.core.command.CommandManager
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.cos
import kotlin.math.sign

class AutoAimCommand(
    private val x: DoubleSupplier,
    private val y: DoubleSupplier,
    private val h: DoubleSupplier,
) : Command() {
    val turretAngle: Double
        get() = sign(x.asDouble) * asin(abs(x.asDouble) / ROMParameters.armLength)
    val turretRadToPositionDeviation
        get() = turretAngle * ROMParameters.turretMaxPositionDeviation / ROMParameters.turretMaxAngleDeviation
    val extensionLength
        get() = y.asDouble - ROMParameters.armLength * cos(turretAngle)
    val extensionLengthToTicks
        get() = extensionLength * ROMParameters.ticksPerCm

    val timer = ElapsedTime()

    override val subsystems: Set<Subsystem>
        get() = setOf(IntakeSubsystem)

    override val isDone: Boolean
        get() = timer.seconds() >= 0.5

    var lastDone = false

    override fun start() {
        val armPosition = IntakeSubsystem.ARM_CENTER - turretRadToPositionDeviation
        val rotation = if (h.asDouble == 0.0) {
            if (armPosition <= (IntakeSubsystem.ARM_LEFT - 0.12)) IntakeSubsystem.RotationState.RIGHT
            else if (armPosition >= (IntakeSubsystem.ARM_RIGHT + 0.12)) IntakeSubsystem.RotationState.LEFT
            else IntakeSubsystem.RotationState.HORIZONTAL
        } else {
            if (armPosition <= (IntakeSubsystem.ARM_LEFT - 0.12)) IntakeSubsystem.RotationState.LEFT
            else if (armPosition >= (IntakeSubsystem.ARM_RIGHT + 0.12)) IntakeSubsystem.RotationState.RIGHT
            else IntakeSubsystem.RotationState.VERTICAL
        }

        IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.EXTENDING)()
        IntakeSubsystem.setExtendoPosition(extensionLengthToTicks.coerceAtMost(IntakeSubsystem.EXTENDO_OUT))()
        IntakeSubsystem.setArmPosition(armPosition)()
        IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.DOWN)()
        IntakeSubsystem.setRotation(rotation)()

        CommandManager.scheduleCommands()

        timer.reset()
        lastDone = false
    }

    override fun update() {
        if (IntakeSubsystem.extendoController.atTarget() != lastDone) {
            if (IntakeSubsystem.extendoController.atTarget())
                timer.reset()

            lastDone = IntakeSubsystem.extendoController.atTarget()
        }
    }
}