package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand
import com.rowanmcalpin.nextftc.ftc.OpModeData
import com.rowanmcalpin.nextftc.ftc.OpModeData.telemetry
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D
import org.firstinspires.ftc.teamcode.util.MotorGroup.setBrakeMode
import kotlin.math.abs
import kotlin.math.max

object DriveSubsystem : Subsystem() {
    private lateinit var motors: List<MotorEx>

    private var maxSpeed: Double = 1.0

    override fun initialize() {
        motors = arrayOf("leftFront", "rightFront", "leftBack", "rightBack")
            .map { id -> MotorEx(id) }
        motors[0].direction = DcMotorSimple.Direction.REVERSE
        motors[2].direction = DcMotorSimple.Direction.REVERSE
        motors.setBrakeMode(DcMotor.ZeroPowerBehavior.BRAKE)
    }

    override val defaultCommand: Command
        get() = teleOp(OpModeData.gamepad1)

    private fun setDriveVector(vector: Vector2D, turn: Double) {
        val forward = vector.x
        val strafe = vector.y

        val denominator = max(abs(forward) + abs(strafe) + abs(turn), 1.0)
        var powers = listOf(
            forward + turn - strafe,
            forward - turn + strafe,
            forward + turn + strafe,
            forward - turn - strafe
        )
        powers = powers.map { power -> power / denominator }

        var ma = 1.0
        for (i in powers.indices) {
            ma = max(ma, abs(powers[i]))
        }
        powers = powers.map { power -> power / ma }

        motors.forEachIndexed { index, motor -> motor.power = powers[index] }
    }

    private fun teleOp(gamepad: Gamepad?) = LambdaCommand()
        .setUpdate {
            if (gamepad == null) return@setUpdate

            var fwd = (-gamepad.left_stick_y).toDouble()
            var stf = (-gamepad.left_stick_x).toDouble() * 1.1
            var rot = gamepad.right_stick_x.toDouble() * 0.7

            fwd *= abs(fwd) * maxSpeed
            stf *= abs(stf) * maxSpeed
            rot *= abs(rot) * (maxSpeed * 2.5).coerceAtMost(1.0)

            var moveVector = Vector2D(fwd, stf)
            if (moveVector.norm <= 0.05)
                moveVector = moveVector.zero

            setDriveVector(moveVector, rot)

            return@setUpdate
        }

    override fun periodic() {
        telemetry.addData("Drive Max Speed", maxSpeed)
    }

    fun setMaxSpeed(speed: Double) = InstantCommand { maxSpeed = speed }
}