
from dataclasses import asdict

from communication import protocol
from communication.typed_protocol import InjectStuckJoint


def inject_stuck_joints():
    return {
    protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.INJECT_FAULT,
    protocol.CtrlMsgKeys.FAULT_TYPE: protocol.FaultTypes.STUCK_JOINT,
    protocol.CtrlMsgKeys.JOINTS: [0, 1, 2],
    }


def inject_typed():
    return asdict(
        InjectStuckJoint(
            joints=[0, 1, 2]
        )
    )

print(inject_stuck_joints())
print(inject_typed())



    