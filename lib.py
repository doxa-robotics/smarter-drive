from vex import (BrakeType, DirectionType, DistanceUnits, DriveTrain, Gps,
                 Gyro, Inertial, Motor, MotorGroup, RotationUnits,
                 TemperatureUnits, TurnType, VelocityPercentUnits,
                 VelocityUnits, vexnumber)


class SmarterDrive:
    """
    VEX's default [SmartDrive], but with smarter PID, etc.
    Assumes the class types are correct, unlike vex.py which does runtime 
    checking. Please make sure you're using a linter like Pylance.
    """

    _target_heading: vexnumber
    turn_threshold: vexnumber
    turn_aggression: vexnumber
    drive_velocity: vexnumber
    external_gear_ratio: vexnumber
    drive_velocity_units: VelocityUnits.VelocityUnits
    stopping: BrakeType.BrakeType
    wheel_travel: vexnumber  # in mm
    gyro: Gyro | Inertial | Gps

    def __init__(
            self,
            lm: MotorGroup | Motor,
            rm: MotorGroup | Motor,
            g: Gyro | Inertial | Gps,
            wheel_travel: vexnumber = 300,
            units: DistanceUnits.DistanceUnits = DistanceUnits.MM,
            externalGearRatio: vexnumber = 1.0,
            turn_threshold: vexnumber = 1,
            turn_aggression: vexnumber = 1.0,
            drive_velocity: vexnumber = 50,
            drive_velocity_units: VelocityUnits.VelocityUnits = VelocityUnits.PERCENT,
            stopping: BrakeType.BrakeType = BrakeType.BRAKE):
        self.target_heading = g.heading()
        self.turn_aggression = turn_aggression
        self.turn_threshold = turn_threshold
        self.drive_velocity = drive_velocity
        self.drive_velocity_units = drive_velocity_units
        self.stopping = stopping
        self.lm = lm
        self.rm = rm
        self.gyro = g
        self.external_gear_ratio = externalGearRatio
        self.wheel_travel = SmarterDrive._distance_to_mm(wheel_travel, units)

    @staticmethod
    def _distance_to_mm(distance: vexnumber, units: DistanceUnits.DistanceUnits):
        match units:
            case DistanceUnits.IN:
                return distance * 25.4
            case DistanceUnits.CM:
                return distance * 10
            case DistanceUnits.MM:
                return distance
            case _:
                raise Exception(
                    "this is what you get from using vex: enums which aren't.")

    def drive(
            self,
            direction: DirectionType.DirectionType,
            velocity: vexnumber | None = None,
            units: VelocityUnits.VelocityUnits = VelocityUnits.RPM):
        self.lm.spin(direction, (velocity if velocity != None else self.drive_velocity_)*self.external_gear_ratio,  # type:ignore
                     units if velocity != None else self.drive_velocity_units)
        self.rm.spin(direction, (velocity if velocity != None else self.drive_velocity)*self.external_gear_ratio,  # type:ignore
                     units if velocity != None else self.drive_velocity_units)

    def drive_for(
            self,
            direction: DirectionType.DirectionType,
            distance: vexnumber,
            units: DistanceUnits.DistanceUnits,
            velocity: vexnumber | None = None,
            units_v: VelocityUnits.VelocityUnits = VelocityUnits.RPM,
            wait: bool = True):
        revolutions = SmarterDrive._distance_to_mm(
            distance, units) / self.wheel_travel
        self.lm.spin_for(direction, revolutions*self.external_gear_ratio, RotationUnits.REV, (velocity if velocity !=  # type:ignore
                         None else self.drive_velocity)*self.external_gear_ratio, units_v if velocity != None else self.drive_velocity_units, wait)
        self.rm.spin_for(direction, revolutions*self.external_gear_ratio, RotationUnits.REV, (velocity if velocity !=  # type:ignore
                         None else self.drive_velocity)*self.external_gear_ratio, units_v if velocity != None else self.drive_velocity_units, wait)

    def turn(
            self,
            direction: TurnType.TurnType,
            velocity: vexnumber | None = None,
            units: VelocityPercentUnits = VelocityUnits.RPM):
        real_velocity = ((velocity if velocity != None else self.drive_velocity) if direction == TurnType.RIGHT else -
                         (velocity if velocity != None else self.drive_velocity))*self.external_gear_ratio
        self.lm.spin(DirectionType.FORWARD, real_velocity,  # type:ignore
                     units if velocity != None else self.drive_velocity_units)
        self.rm.spin(DirectionType.REVERSE, real_velocity,  # type:ignore
                     units if velocity != None else self.drive_velocity_units)

    def turn_for(
            self,
            direction: TurnType.TurnType,
            angle: vexnumber
    ):
        """ Turn for the specified relative angle, in degrees. """
        self._target_heading += angle if direction == TurnType.RIGHT else -angle
        heading = self.gyro.heading()
        while abs(self._target_heading - heading) >= self.turn_threshold:
            difference = (self._target_heading -
                          heading) % 360  # NEEDS DEBUGGING
            if difference > 180:
                difference = difference - 360
            velocity = difference * self.turn_aggression
            self.turn(TurnType.RIGHT, velocity *
                      self.external_gear_ratio, VelocityUnits.PERCENT)

    def is_moving(self) -> bool:
        return self.lm.is_spinning() or self.rm.is_spinning()  # type:ignore

    def is_done(self) -> bool:
        return not self.is_moving()

    def stop(self, mode: BrakeType.BrakeType | None = None):
        brake = mode if mode != None else self.stopping
        self.lm.stop(brake)  # type:ignore
        self.rm.stop(brake)  # type:ignore

    def max_temperature(self, units: TemperatureUnits.TemperatureUnits = TemperatureUnits.CELSIUS):
        motors: list[Motor] = (self.lm._motors if isinstance(self.lm, MotorGroup) else [  # type:ignore
            self.lm]) + (self.rm._motors if isinstance(self.rm, MotorGroup) else [self.rm])  # type:ignore
        max: vexnumber = 0
        for motor in motors:
            temp = motor.temperature(units)  # type:ignore
            if temp > max:
                max = temp
        return max
