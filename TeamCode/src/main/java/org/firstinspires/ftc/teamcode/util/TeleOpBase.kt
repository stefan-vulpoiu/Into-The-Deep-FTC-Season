package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.util.ElapsedTime
import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode
import com.rowanmcalpin.nextftc.ftc.OpModeData
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx
import kotlin.math.roundToInt

open class TeleOpBase(vararg sbst: Subsystem = arrayOf()) : NextFTCOpMode(*sbst) {
    lateinit var driver1: GamepadEx
    lateinit var driver2: GamepadEx

    private var runtime: ElapsedTime = ElapsedTime()
    private var loopTime: ElapsedTime = ElapsedTime()

    override fun onInit() {
        this.telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.msTransmissionInterval = 10
        OpModeData.telemetry = this.telemetry

        driver1 = gamepadManager.gamepad1
        driver2 = gamepadManager.gamepad2

        runtime.reset()
        loopTime.reset()

        this.initialize()
    }

    override fun onUpdate() {
        this.update()

        telemetry.addData("Runtime", "${runtime.seconds().roundToInt()} seconds")
        telemetry.addData("Loop Time", "${loopTime.milliseconds().roundToInt()} ms")
        telemetry.addLine()

        loopTime.reset()

        telemetry.update()
    }

    open fun initialize() {}

    open fun update() {}
}