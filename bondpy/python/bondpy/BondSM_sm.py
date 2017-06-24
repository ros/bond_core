# ex: set ro:
# DO NOT EDIT.
# generated by smc (http://smc.sourceforge.net/)
# from file : BondSM_sm.sm

from smclib import statemap


class BondSMState(statemap.State):

    def Entry(self, fsm):
        pass

    def Exit(self, fsm):
        pass

    def ConnectTimeout(self, fsm):
        self.Default(fsm)

    def Die(self, fsm):
        self.Default(fsm)

    def DisconnectTimeout(self, fsm):
        self.Default(fsm)

    def HeartbeatTimeout(self, fsm):
        self.Default(fsm)

    def SisterAlive(self, fsm):
        self.Default(fsm)

    def SisterDead(self, fsm):
        self.Default(fsm)

    def Default(self, fsm):
        msg = "\n\tState: %s\n\tTransition: %s" % (
            fsm.getState().getName(), fsm.getTransition())
        raise statemap.TransitionUndefinedException(msg)

class SM_Default(BondSMState):
    pass

class SM_WaitingForSister(SM_Default):

    def ConnectTimeout(self, fsm):
        ctxt = fsm.getOwner()
        fsm.getState().Exit(fsm)
        fsm.clearState()
        try:
            ctxt.Death()
        finally:
            fsm.setState(SM.Dead)
            fsm.getState().Entry(fsm)

    def Die(self, fsm):
        ctxt = fsm.getOwner()
        fsm.getState().Exit(fsm)
        fsm.clearState()
        try:
            ctxt.Death()
        finally:
            fsm.setState(SM.Dead)
            fsm.getState().Entry(fsm)

    def SisterAlive(self, fsm):
        ctxt = fsm.getOwner()
        fsm.getState().Exit(fsm)
        fsm.clearState()
        try:
            ctxt.Connected()
        finally:
            fsm.setState(SM.Alive)
            fsm.getState().Entry(fsm)

    def SisterDead(self, fsm):
        ctxt = fsm.getOwner()
        fsm.getState().Exit(fsm)
        fsm.clearState()
        try:
            ctxt.Connected()
            ctxt.SisterDied()
            ctxt.Death()
        finally:
            fsm.setState(SM.Dead)
            fsm.getState().Entry(fsm)

class SM_Alive(SM_Default):

    def Die(self, fsm):
        ctxt = fsm.getOwner()
        fsm.getState().Exit(fsm)
        fsm.clearState()
        try:
            ctxt.StartDying()
        finally:
            fsm.setState(SM.AwaitSisterDeath)
            fsm.getState().Entry(fsm)

    def HeartbeatTimeout(self, fsm):
        ctxt = fsm.getOwner()
        fsm.getState().Exit(fsm)
        fsm.clearState()
        try:
            ctxt.Death()
        finally:
            fsm.setState(SM.Dead)
            fsm.getState().Entry(fsm)

    def SisterAlive(self, fsm):
        ctxt = fsm.getOwner()
        fsm.getState().Exit(fsm)
        fsm.clearState()
        try:
            ctxt.Heartbeat()
        finally:
            fsm.setState(SM.Alive)
            fsm.getState().Entry(fsm)

    def SisterDead(self, fsm):
        ctxt = fsm.getOwner()
        fsm.getState().Exit(fsm)
        fsm.clearState()
        try:
            ctxt.SisterDied()
            ctxt.Death()
        finally:
            fsm.setState(SM.Dead)
            fsm.getState().Entry(fsm)

class SM_AwaitSisterDeath(SM_Default):

    def Die(self, fsm):
        fsm.getState().Exit(fsm)
        fsm.setState(SM.AwaitSisterDeath)
        fsm.getState().Entry(fsm)

    def DisconnectTimeout(self, fsm):
        ctxt = fsm.getOwner()
        fsm.getState().Exit(fsm)
        fsm.clearState()
        try:
            ctxt.Death()
        finally:
            fsm.setState(SM.Dead)
            fsm.getState().Entry(fsm)

    def HeartbeatTimeout(self, fsm):
        fsm.getState().Exit(fsm)
        fsm.setState(SM.AwaitSisterDeath)
        fsm.getState().Entry(fsm)

    def SisterAlive(self, fsm):
        fsm.getState().Exit(fsm)
        fsm.setState(SM.AwaitSisterDeath)
        fsm.getState().Entry(fsm)

    def SisterDead(self, fsm):
        ctxt = fsm.getOwner()
        fsm.getState().Exit(fsm)
        fsm.clearState()
        try:
            ctxt.Death()
        finally:
            fsm.setState(SM.Dead)
            fsm.getState().Entry(fsm)

class SM_Dead(SM_Default):

    def ConnectTimeout(self, fsm):
        fsm.getState().Exit(fsm)
        fsm.setState(SM.Dead)
        fsm.getState().Entry(fsm)

    def Die(self, fsm):
        fsm.getState().Exit(fsm)
        fsm.setState(SM.Dead)
        fsm.getState().Entry(fsm)

    def DisconnectTimeout(self, fsm):
        fsm.getState().Exit(fsm)
        fsm.setState(SM.Dead)
        fsm.getState().Entry(fsm)

    def HeartbeatTimeout(self, fsm):
        fsm.getState().Exit(fsm)
        fsm.setState(SM.Dead)
        fsm.getState().Entry(fsm)

    def SisterAlive(self, fsm):
        fsm.getState().Exit(fsm)
        fsm.setState(SM.Dead)
        fsm.getState().Entry(fsm)

    def SisterDead(self, fsm):
        fsm.getState().Exit(fsm)
        fsm.setState(SM.Dead)
        fsm.getState().Entry(fsm)

class SM(object):

    WaitingForSister = SM_WaitingForSister('SM.WaitingForSister', 0)
    Alive = SM_Alive('SM.Alive', 1)
    AwaitSisterDeath = SM_AwaitSisterDeath('SM.AwaitSisterDeath', 2)
    Dead = SM_Dead('SM.Dead', 3)
    Default = SM_Default('SM.Default', -1)

class BondSM_sm(statemap.FSMContext):

    def __init__(self, owner):
        statemap.FSMContext.__init__(self, SM.WaitingForSister)
        self._owner = owner

    def __getattr__(self, attrib):
        def trans_sm(*arglist):
            self._transition = attrib
            getattr(self.getState(), attrib)(self, *arglist)
            self._transition = None
        return trans_sm

    def enterStartState(self):
        self._state.Entry(self)

    def getOwner(self):
        return self._owner

# Local variables:
#  buffer-read-only: t
# End:
