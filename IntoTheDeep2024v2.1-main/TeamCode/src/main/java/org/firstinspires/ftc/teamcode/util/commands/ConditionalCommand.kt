package org.firstinspires.ftc.teamcode.util.commands

import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.core.command.CommandManager
import com.rowanmcalpin.nextftc.core.command.utility.NullCommand

class ConditionalCommand(
    private val condition: () -> Boolean,
    private val trueCommand: () -> Command,
    private val falseCommand: (() -> Command) = { NullCommand() },
) : Command() {
    private var selectedCommand: Command? = null

    override val isDone: Boolean
        get() {
            if (selectedCommand?.isDone == true) {
                return selectedCommand!!.isDone
            }
            return false
        }

    override fun start() {
        selectedCommand = if (condition()) {
            trueCommand()
        } else {
            falseCommand()
        }

        CommandManager.scheduleCommand(selectedCommand!!)
        CommandManager.scheduleCommands()
    }
}