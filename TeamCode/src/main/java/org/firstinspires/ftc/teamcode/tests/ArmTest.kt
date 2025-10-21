package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.rowanmcalpin.nextftc.ftc.OpModeData
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.util.InterpolatedServo
import org.firstinspires.ftc.teamcode.util.TeleOpBase

@Config("Arm Test")
@TeleOp(name = "Arm Test", group = "Tests")
class ArmTest : TeleOpBase() {
    companion object {
        @JvmField
        var POSITION = 0.0
    }

    private lateinit var armLeft: InterpolatedServo
    private lateinit var armRight: InterpolatedServo

    override fun initialize() {
        armLeft = InterpolatedServo(OpModeData.hardwareMap, "arm_left")
        armLeft.setInverted(false)
        armLeft.generatePositions(*OuttakeSubsystem.armLeftPositions)

        armRight = InterpolatedServo(OpModeData.hardwareMap, "arm_right")
        armRight.setInverted(true)
        armRight.generatePositions(*OuttakeSubsystem.armRightPositions)
    }

    override fun update() {
        armLeft.position = POSITION
        armRight.position = POSITION
    }
}