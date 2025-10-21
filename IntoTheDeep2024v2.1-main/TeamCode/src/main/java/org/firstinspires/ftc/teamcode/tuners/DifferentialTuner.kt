package org.firstinspires.ftc.teamcode.tuners

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.util.InterpolatedServo
import org.firstinspires.ftc.teamcode.util.TeleOpBase

@Config("Differential Tuner")
@TeleOp(name = "Differential Tuner", group = "Tuners")
class DifferentialTuner : TeleOpBase() {
    companion object {
        @JvmField
        var LEFT_POS = 0.5

        @JvmField
        var LEFT_INVERT = true

        @JvmField
        var RIGHT_POS = 0.5

        @JvmField
        var RIGHT_INVERT = false

        @JvmField
        var INT_LIFT_POS = 0.8

        @JvmField
        var ROTATION = 0.37
    }

    private lateinit var diffLeft: InterpolatedServo
    private lateinit var diffRight: InterpolatedServo
    private lateinit var lift: InterpolatedServo
    private lateinit var rotate: InterpolatedServo

    override fun initialize() {
        diffLeft = InterpolatedServo(hardwareMap, "int_left", 1.0)
        diffRight = InterpolatedServo(hardwareMap, "int_right", 1.0)
        lift = InterpolatedServo(hardwareMap, "int_lift", 1.0)
        lift.setInverted(true)
        rotate = InterpolatedServo(hardwareMap, "int_rotate", 1.0)
    }

    override fun update() {
        lift.position = INT_LIFT_POS

        diffLeft.position = LEFT_POS
        diffRight.position = RIGHT_POS

        diffLeft.setInverted(LEFT_INVERT)
        diffRight.setInverted(RIGHT_INVERT)

        rotate.position = ROTATION
    }
}