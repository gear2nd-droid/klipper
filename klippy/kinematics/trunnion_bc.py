# Code for handling the kinematics of Trunnion BC robots
#
# Copyright (C) 2023-  @_geek_gear_ <gear2nd.droid@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper
import math

class TrunnionBcKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.toolhead = toolhead
        self.offset_x = config.getfloat('offset_x')
        self.offset_y = config.getfloat('offset_y')
        self.offset_z = config.getfloat('offset_z')
        self.offset_a = math.radians(config.getfloat('offset_a'))
        self.offset_b = math.radians(config.getfloat('offset_b'))
        self.offset_c = math.radians(config.getfloat('offset_c'))
        # Setup axis rails
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyzbc']
        for rail, axis in zip(self.rails, 'xyzbc'):
            rail.setup_itersolve('trunnion_bc_stepper_alloc', axis.encode(), self.offset_x, self.offset_y, self.offset_z, self.offset_a, self.offset_b, self.offset_c)
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        self.printer.register_event_handler("stepper_enable:motor_off",
                                            self._motor_off)
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 5
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)
        # trunnion
        self.after_homing_z_position = config.getfloat('after_homing_z_position')
        self.before_homing_z_move_x = config.getfloat('before_homing_z_move_x')
        self.before_homing_z_move_y = config.getfloat('before_homing_z_move_y')
        hx = self.rails[0].get_homing_info()
        hy = self.rails[1].get_homing_info()
        hz = self.rails[2].get_homing_info()
        hb = self.rails[3].get_homing_info()
        hc = self.rails[4].get_homing_info()
        self.home_position = [hx.position_endstop, hy.position_endstop, hz.position_endstop, 0., hb.position_endstop, hc.position_endstop]
        # for debug
        self.gcode = self.printer.lookup_object('gcode')
    def get_steppers(self):
        rails = self.rails
        return [s for rail in rails for s in rail.get_steppers()]
    def calc_position(self, stepper_positions):
        pos = self.home_position
        return pos
    def set_position(self, newpos, homing_axes):
        for i in (0, 1, 2, 3, 4):
            self.rails[i].set_position(newpos)
            if i in homing_axes:
                self.limits[i] = self.rails[i].get_range()
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limits[2] = (1.0, -1.0)
    def _home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None, None, None, None]
        if axis == 0:
            homepos[axis] = hi.position_endstop
        elif axis == 1:
            homepos[axis] = hi.position_endstop
        elif axis == 2:
            homepos[axis] = hi.position_endstop
        elif axis == 4:
            homepos[axis] = hi.position_endstop
        elif axis == 5:
            homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 2.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 2.5 * (position_max - hi.position_endstop)
        # Perform homing
        #self.gcode.respond_info('forcepos:{0}, homepos:{1}'.format(forcepos, homepos))
        homing_state.home_rails([rail], forcepos, homepos)
    def home(self, homing_state):
        toolhead = self.printer.lookup_object('toolhead')
        self._home_axis(homing_state, 0, self.rails[0])
        self._home_axis(homing_state, 1, self.rails[1])
        hz = self.rails[2].get_homing_info()
        pos = toolhead.get_position()
        pos[0] = pos[0] + self.before_homing_z_move_x
        pos[1] = pos[1] + self.before_homing_z_move_y
        toolhead.manual_move(pos, hz.speed) 
        self._home_axis(homing_state, 2, self.rails[2])
        pos = toolhead.get_position()
        pos[2] = self.after_homing_z_position
        toolhead.manual_move(pos, hz.speed) 
        self._home_axis(homing_state, 5, self.rails[4])
        self._home_axis(homing_state, 4, self.rails[3])
        home_pos = (self.home_position[0], self.home_position[1], self.home_position[2], self.home_position[3], self.home_position[4], self.home_position[5], 0.0)
        self.toolhead.set_position(home_pos)
        pos = toolhead.get_position()
        pos[0] = 0.0
        pos[1] = 0.0
        pos[3] = 0.0
        pos[4] = 0.0
        toolhead.manual_move(pos, hz.speed) 
    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 3
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return TrunnionBcKinematics(toolhead, config)
