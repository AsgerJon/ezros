# -*- coding: utf-8 -*-

#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen

import genpy

AuxCommand = type('AuxCommand', (genpy.Message,), {})
BackgroundSound = type('BackgroundSound', (genpy.Message,), {})
CalibrationInfo = type('CalibrationInfo', (genpy.Message,), {})
CmdWheel = type('CmdWheel', (genpy.Message,), {})
DomainState = type('DomainState', (genpy.Message,), {})
Float32Stamped = type('Float32Stamped', (genpy.Message,), {})
ForegroundSound = type('ForegroundSound', (genpy.Message,), {})
Job = type('Job', (genpy.Message,), {})
JobAction = type('JobAction', (genpy.Message,), {})
JobEvent = type('JobEvent', (genpy.Message,), {})
JobList = type('JobList', (genpy.Message,), {})
JobState = type('JobState', (genpy.Message,), {})
JobTask = type('JobTask', (genpy.Message,), {})
KeyData = type('KeyData', (genpy.Message,), {})
KeyDataArray = type('KeyDataArray', (genpy.Message,), {})
KeyValueOption = type('KeyValueOption', (genpy.Message,), {})
Lamp = type('Lamp', (genpy.Message,), {})
Magnetometer = type('Magnetometer', (genpy.Message,), {})
OdrivePid = type('OdrivePid', (genpy.Message,), {})
PolygonArrayStamped = type('PolygonArrayStamped', (genpy.Message,), {})
PoseNamed = type('PoseNamed', (genpy.Message,), {})
PumpCommand = type('PumpCommand', (genpy.Message,), {})
SprayStatus = type('SprayStatus', (genpy.Message,), {})
StringStamped = type('StringStamped', (genpy.Message,), {})
StringTimed = type('StringTimed', (genpy.Message,), {})
SystemEvent = type('SystemEvent', (genpy.Message,), {})
SystemState = type('SystemState', (genpy.Message,), {})
UserInfo = type('UserInfo', (genpy.Message,), {})
