package org.firstinspires.ftc.teamcode.tuners

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.util.InterpolatedServo
import org.firstinspires.ftc.teamcode.util.TeleOpBase

@Config("Double Servo Tuner")
@TeleOp(name = "Double Servo Tuner", group = "Tuners")
class DoubleServoTuner : TeleOpBase() {
    companion object {
        @JvmField
        var FIRST_SERVO_NAME = "arm_left"

        @JvmField
        var FIRST_INVERT = false

        @JvmField
        var FIRST_MAX_ANGLE = 1.0

        @JvmField
        var FIRST_POSITION = 0.0

        @JvmField
        var SECOND_SERVO_NAME = "arm_right"

        @JvmField
        var SECOND_INVERT = true

        @JvmField
        var SECOND_MAX_ANGLE = 1.0

        @JvmField
        var SECOND_POSITION = 0.0
    }

    private lateinit var servo1: InterpolatedServo
    private lateinit var servo2: InterpolatedServo

    override fun initialize() {
        servo1 = InterpolatedServo(hardwareMap, FIRST_SERVO_NAME, FIRST_MAX_ANGLE)
        servo2 = InterpolatedServo(hardwareMap, SECOND_SERVO_NAME, SECOND_MAX_ANGLE)
    }

    override fun update() {
        servo1.position = FIRST_POSITION
        servo1.setInverted(FIRST_INVERT)

        servo2.position = SECOND_POSITION
        servo2.setInverted(SECOND_INVERT)
    }
}