"""Constant and messages definition for MT communication."""


class Scenarios:
    """Scenario ID (for MTi-1 series)"""
    ID2Lable = {50: 'General',
                51: 'high_mag_dep',
                52: 'dynamic',
                53: 'north_reference',
                54: 'vru_general'}


class MID:
    """State setting messages"""
    # Switch to config state
    GoToConfig = '\x30'
    # Switch to measurement state
    GoToMeasurement = '\x10'

    """Device specific messages"""
    # Restore factory defaults
    RestoreFactoryDef = '\x0E'
    # Output configuration (MTi-10/100 series only), N*4 bytes
    OutputConfiguration = '\xC0'
    # Latitude, Longitude and Altitude for local declination and gravity
    SetLatLonAlt = '\x6E'  # Set and Req use the same MID

    """Data message"""
    # Newer data packet (MTi-1/10/100 series)
    MTData2 = '\x36'

    """XKF Filter messages"""
    # Request the available XKF scenarios on the device
    ReqAvailableScenarios = '\x62'
    # Current XKF scenario
    SetCurrentScenario = '\x64'  # Set and Req use the same MID


class XDIGroup:
    """Values for the XDI groups."""
    OrientationData = 0x2030  # Euler angles
    Acceleration = 0x4030  # free acceleration
    AngularVelocity = 0x8020  # rate of turn

    Req = {'Ori':     OrientationData,
           'Acc_lin': Acceleration,
           'Vel_ang': AngularVelocity}

    # FIXME: At present, only 'ENU' convention works
    # Hence, the coordinate tranformation is done manually.
    coorSys = {'ENU': 0x0000,
               'NED': 0x0004,
               'NWU': 0x0008}

    dataFormat = {'f': 0x0000,
                  'd': 0x0003}
