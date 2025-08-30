package org.firstinspires.ftc.teamcode.commandbase;

import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;

public class CommandUtil {
    public static Lambda proxiedCommand(Command command){
        return new Lambda("Proxied " + command.toString())
                .setInit(command::schedule)
                .setFinish(() -> !Mercurial.isScheduled(command));
    }
    public static Lambda instant(Runnable runnable){
        return new Lambda("instant")
                .setRunStates(Wrapper.OpModeState.INIT, Wrapper.OpModeState.ACTIVE, Wrapper.OpModeState.STOPPED)
                .setInit(runnable);
    }
}
