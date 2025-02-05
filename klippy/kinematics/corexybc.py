# Code for handling the kinematics of corexy robots
#
# Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper

def linear_fit(x, y):
    if len(x) != len(y):
        raise ValueError('Due to mismatched array lengths, linear function fitting is not possible.')
    N = len(x)
    sum_x = sum(x)
    sum_y = sum(y)
    sum_x2 = sum(xi**2 for xi in x)
    sum_xy = sum(xi * yi for xi, yi in zip(x, y))
    a = (N * sum_xy - sum_x * sum_y) / (N * sum_x2 - sum_x**2)
    b = (sum_y - a * sum_x) / N
    return a, b

class CoreXYBCKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.toolhead = toolhead
        # adjust param before
        self.now_calibrate = False
        self.check_move_speed = config.getfloat('check_move_speed')
        self.check_move_safe = config.getfloat('check_move_safe')
        self.adjust_a = config.getfloat('adjust_a')
        self.check_a_move = config.get('check_a_move')
        # Setup axis rails
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyzbc']
        #for s in self.rails[1].get_steppers():
        #    self.rails[0].get_endstops()[0][0].add_stepper(s)
        #for s in self.rails[0].get_steppers():
        #    self.rails[1].get_endstops()[0][0].add_stepper(s)
        for rail, axis in zip(self.rails, 'xyzbc'):
            rail.setup_itersolve('corexybc_stepper_alloc', axis.encode(), \
                                0.0, 0.0, 0.0, self.adjust_a, 0.0, 0.0)
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)
        # adjust param after
        self.adjust_b = self.rails[3].position_endstop
        self.check_b_move = config.get('check_b_move')
        self.adjust_c = self.rails[4].position_endstop
        self.check_c_move = config.get('check_c_move')
        self.adjust_x = self.rails[0].position_endstop
        self.check_x_move = config.get('check_x_move')
        self.adjust_y = self.rails[1].position_endstop
        self.check_y_move = config.get('check_y_move')
        # max speed
        self.max_speed_x = config.getfloat('max_speed_x')
        self.max_speed_y = config.getfloat('max_speed_y')
        self.max_speed_z = config.getfloat('max_speed_z')
        self.max_speed_a = config.getfloat('max_speed_a')
        self.max_speed_b = config.getfloat('max_speed_b')
        self.max_speed_c = config.getfloat('max_speed_c')
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 5
        ranges = [r.get_range() for r in self.rails]
        buf = (ranges[0][0], ranges[1][0], ranges[2][0], 0, ranges[3][0], ranges[4][0], )
        self.axes_min = toolhead.Coord(x=ranges[0][0], y=ranges[1][0], z=ranges[2][0], \
                                        a=0, b=ranges[3][0], c=ranges[4][0], e=0.)
        self.axes_max = toolhead.Coord(x=ranges[0][1], y=ranges[1][1], z=ranges[2][1], \
                                        a=0, b=ranges[3][1], c=ranges[4][1], e=0.)
        # corexybc
        self.probe = self.printer.lookup_object('probe')
        self.probe_x_offset = self.probe.x_offset
        self.probe_y_offset = self.probe.y_offset
        self.probe_z_offset = self.probe.z_offset
        self.gcode = self.printer.lookup_object('gcode')
        hx = self.rails[0].get_homing_info()
        hy = self.rails[1].get_homing_info()
        hz = self.rails[2].get_homing_info()
        hb = self.rails[3].get_homing_info()
        hc = self.rails[4].get_homing_info()
        self.home_position = [hx.position_endstop, hy.position_endstop, hz.position_endstop, 0., hb.position_endstop, hc.position_endstop]
        self.center_x = config.getfloat('center_x')
        self.center_y = config.getfloat('center_y')
        self.bed_diameter = config.getfloat('bed_diameter')
        self.nozzle_outer_diameter = config.getfloat('nozzle_outer_diameter')
        self.gcode.register_command('A_CALIBRATE', self.cmd_A_CALIBRATE,
                                    desc=self.cmd_A_CALIBRATE_help)
        self.gcode.register_command('B_CALIBRATE', self.cmd_B_CALIBRATE,
                                    desc=self.cmd_B_CALIBRATE_help)
        self.gcode.register_command('C_CALIBRATE', self.cmd_C_CALIBRATE,
                                    desc=self.cmd_C_CALIBRATE_help)
        self.gcode.register_command('X_CALIBRATE', self.cmd_X_CALIBRATE,
                                    desc=self.cmd_X_CALIBRATE_help)
        self.gcode.register_command('Y_CALIBRATE', self.cmd_Y_CALIBRATE,
                                    desc=self.cmd_Y_CALIBRATE_help)
        
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    def calc_position(self, stepper_positions):
        pos = [stepper_positions[rail.get_name()] for rail in self.rails]
        px = pos[0]
        py = pos[1]
        pz = pos[2]
        pb = pos[3]
        pc = pos[4]
        kx = 0.5 * (px + py)
        ky = 0.5 * (px - py)
        kz = pz
        bx = kx
        by = ky * math.cos(-self.adjust_a / 180.0 * math.pi) \
            - kz * math.sin(-self.adjust_a / 180.0 * math.pi)
        bz = ky * math.sin(-self.adjust_a / 180.0 * math.pi) \
            + kz * math.cos(-self.adjust_a / 180.0 * math.pi)
        return [bx, by, bz, 0, pb, pc]
    def commanded_pos_to_real_pos(self, commanded_pos):
        flag = False
        for i in range(3):
            if self.limits[i][0] > self.limits[i][1]:
                flag = True
        #if flag:
        #    return commanded_pos
        px = commanded_pos[0]
        py = commanded_pos[1]
        pz = commanded_pos[2]
        pb = commanded_pos[4]
        pc = commanded_pos[5]
        pe = commanded_pos[6]
        bx = px
        by = py * math.cos(-self.adjust_a / 180.0 * math.pi) - pz * math.sin(-self.adjust_a / 180.0 * math.pi)
        bz = py * math.sin(-self.adjust_a / 180.0 * math.pi) + pz * math.cos(-self.adjust_a / 180.0 * math.pi)
        return [0.5 * (bx + by), 0.5 * (bx - by), bz, 0, pb, pc, pe]
	
    def set_position(self, newpos, homing_axes):
        for i in range(5):
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
        elif axis == 3:
            homepos[axis] = hi.position_endstop
        elif axis == 4:
            homepos[axis] = hi.position_endstop
        elif axis == 5:
            homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            if axis == 5:
                forcepos[axis] -= 360.0
            else:
                forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            if axis == 5:
                forcepos[axis] += 360.0
            else:
                forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        homing_state.home_rails([rail], forcepos, homepos)
    def home(self, homing_state):
        self.limits[3] = self.rails[3].get_range()
        self.limits[4] = self.rails[4].get_range()
        self._home_axis(homing_state, 2, self.rails[2])
        self._home_axis(homing_state, 0, self.rails[0])
        self._home_axis(homing_state, 1, self.rails[1])
        self._home_axis(homing_state, 4, self.rails[3])
        self._home_axis(homing_state, 5, self.rails[4])
        # g92
        #self.gcode.run_script_from_command('G92 C0\n') 
        pos = self.toolhead.get_position()
        pos[0] = self.center_x
        pos[1] = self.center_y
        pos[4] = 0.0
        pos[5] = 0.0
        hi = self.rails[0].get_homing_info()
        self.toolhead.manual_move(pos, hi.speed)
        self.gcode.run_script_from_command('G92 X0 Y0\n') 
    def _motor_off(self, print_time):
        #self.limits = [(1.0, -1.0)] * 3
        self.limits = [(1.0, -1.0)] * 5
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
        xpos, ypos, zpos = move.end_pos[:3]
        bpos = move.end_pos[4]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if(move.axes_d[4]):
            if(bpos < limits[3][0] or limits[3][1] < bpos):
                raise move.move_error()        
        if not move.axes_d[2]:
            return
        if(move.axes_d[2]):
            if(self.bed_diameter / 2.0 < abs(xpos)):
                if(xpos > 0): 
                    bed_level = self.bed_diameter / 2.0 * math.sin(-bpos / 180.0 * math.pi)
                else:
                    bed_level = -self.bed_diameter / 2.0 * math.sin(-bpos / 180.0 * math.pi)
            else:
                bed_level = xpos * math.sin(-bpos / 180.0 * math.pi)
            if(zpos < bed_level):
                if self.now_calibrate == False:
                    raise move.move_error()  
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyzbc", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }
    def calc_move_distance(self, start_pos, end_pos):
        div = 10
        dist_target = 0.1
        px = start_pos[0]
        py = start_pos[1]
        pz = start_pos[2]
        ptilt = start_pos[4]
        prot = start_pos[5]
        nx = end_pos[0]
        ny = end_pos[1]
        nz = end_pos[2]
        ntilt = end_pos[4]
        nrot = end_pos[5]
        # start_pos
        brot = math.radians(prot)
        btilt = math.radians(ptilt)
        bx2 = px * math.cos(btilt) - pz * math.sin(btilt)
        by2 = py
        bz2 = px * math.sin(btilt) + pz * math.cos(btilt)
        stx = bx2 * math.cos(-brot) - by2 * math.sin(-brot)
        sty = bx2 * math.sin(-brot) + by2 * math.cos(-brot)
        stz = bz2        
        # calc div
        bx1 = (nx - px) * (0+1) / div + px
        by1 = (ny - py) * (0+1) / div + py
        bz1 = (nz - pz) * (0+1) / div + pz
        brot = (nrot - prot) * (0+1) / div + prot
        btilt = (ntilt - ptilt) * (0+1) / div + ptilt
        brot = math.radians(brot)
        btilt = math.radians(btilt)
        bx2 = bx1 * math.cos(btilt) - bz1 * math.sin(btilt)
        by2 = by1
        bz2 = bx1 * math.sin(btilt) + bz1 * math.cos(btilt)
        bx3 = bx2 * math.cos(-brot) - by2 * math.sin(-brot)
        by3 = bx2 * math.sin(-brot) + by2 * math.cos(-brot)
        bz3 = bz2
        dist = math.sqrt((bx3 - stx)**2 + (by3 - sty)**2 + (bz3 - stz)**2) 
        div = math.ceil(div * dist / dist_target)  
        # calc dist
        dist = 0.0
        bx4 = stx
        by4 = sty
        bz4 = stz
        for i in range(div):
            bx1 = (nx - px) * (i+1) / div + px
            by1 = (ny - py) * (i+1) / div + py
            bz1 = (nz - pz) * (i+1) / div + pz
            brot = (nrot - prot) * (i+1) / div + prot
            btilt = (ntilt - ptilt) * (i+1) / div + ptilt
            brot = math.radians(brot)
            btilt = math.radians(btilt)
            # calc pos
            bx2 = bx1 * math.cos(btilt) - bz1 * math.sin(btilt)
            by2 = by1
            bz2 = bx1 * math.sin(btilt) + bz1 * math.cos(btilt)
            bx3 = bx2 * math.cos(-brot) - by2 * math.sin(-brot)
            by3 = bx2 * math.sin(-brot) + by2 * math.cos(-brot)
            bz3 = bz2
            dist += math.sqrt((bx3 - bx4)**2 + (by3 - by4)**2 + (bz3 - bz4)**2)
            bx4 = bx3
            by4 = by3
            bz4 = bz3
        return dist
        
    cmd_A_CALIBRATE_help = "A axis calibration for coreXYBC"
    def cmd_A_CALIBRATE(self, gcmd):
        kp = 0.5
        self.now_calibrate = True
        gcmd.respond_info('[printer]adjust_a:{0}'.format(self.adjust_a))
        str_y = self.check_a_move.split(",")
        buf_y = [float(str_y[0]), float(str_y[-1])]
        pos = self.toolhead.get_position()
        pos[0] = 0.0
        pos[1] = buf_y[0]
        pos[2] = pos[2]
        pos[3] = 0.0
        pos[4] = 0.0
        pos[5] = 0.0
        self.toolhead.manual_move(pos, self.check_move_speed)
        ret1 = self.probe.run_probe(gcmd)
        pos[1] = buf_y[1]
        pos[2] = ret1[2] + self.check_move_safe
        self.toolhead.manual_move(pos, self.check_move_speed)
        ret2 = self.probe.run_probe(gcmd)
        pos[2] = ret2[2] + self.check_move_safe
        self.toolhead.manual_move(pos, self.check_move_speed)
        self.now_calibrate = False
        new_val = -math.atan2(ret1[2] - ret2[2], buf_y[1] - buf_y[0]) / math.pi * 180.0
        if self.adjust_a != 0.0:
            gcmd.respond_info('Check that the difference between the probed values is less than 0.2.')
            gcmd.respond_info('If the condition is not met, change adjust_a in PRINTER to {0}.'.format(
                self.adjust_a + new_val * kp))
        else:
            gcmd.respond_info('Change adjust_a in [printer] to {0}.'.format(new_val)) 
            
    cmd_B_CALIBRATE_help = "B axis calibration for coreXYBC"
    def cmd_B_CALIBRATE(self, gcmd):
        self.now_calibrate = True
        kp = 0.5
        gcmd.respond_info('[stepper_b]position_endstop:{0}'.format(self.adjust_b))
        str_xy = self.check_b_move.split(",") 
        buf_x = float(str_xy[0])
        buf_y = float(str_xy[1])
        pos = self.toolhead.get_position()
        old_val = 0.0
        cnt = 1
        pos[0] = buf_x
        pos[1] = buf_y
        pos[2] = pos[2]
        pos[3] = 0.0
        pos[4] = old_val
        pos[5] = 0.0
        self.toolhead.manual_move(pos, self.check_move_speed)
        ret1 = self.probe.run_probe(gcmd)
        pos[0] = -buf_x
        pos[2] = ret1[2] + self.check_move_safe
        self.toolhead.manual_move(pos, self.check_move_speed)
        ret2 = self.probe.run_probe(gcmd)
        pos[2] = ret2[2] + self.check_move_safe
        self.toolhead.manual_move(pos, self.check_move_speed)
        delta = ret1[2] - ret2[2]
        new_val = math.atan2(delta, buf_x * 2.0) / math.pi * 180.0 * kp + old_val
        gcmd.respond_info('{0}: {1} / {2} => new_val:{3} , delta:{4}'.format(
            cnt, ret1[2], ret2[2], new_val, delta))
        while(abs(delta) > 0.01):
            old_val = new_val
            cnt = cnt + 1
            pos[0] = buf_x
            pos[2] = ret2[2] + self.check_move_safe
            pos[4] = new_val
            self.toolhead.manual_move(pos, self.check_move_speed)
            ret1 = self.probe.run_probe(gcmd)
            pos[0] = -buf_x
            pos[2] = ret1[2] + self.check_move_safe
            self.toolhead.manual_move(pos, self.check_move_speed)
            ret2 = self.probe.run_probe(gcmd)
            pos[2] = ret2[2] + self.check_move_safe
            self.toolhead.manual_move(pos, self.check_move_speed)
            delta = ret1[2] - ret2[2]
            new_val = math.atan2(delta, buf_x * 2.0) / math.pi * 180.0 * kp + old_val
            gcmd.respond_info('{0}: {1} / {2} => new_val:{3} , delta:{4}'.format(
                cnt, ret1[2], ret2[2], new_val, delta))
        gcmd.respond_info('Change position_endstop in [stepper_b] to {0}.'.format(
            self.adjust_b - old_val)) 
        self.now_calibrate = False
            
    cmd_C_CALIBRATE_help = "C axis calibration for coreXYBC"
    def cmd_C_CALIBRATE(self, gcmd):
        self.now_calibrate = True
        kp = 0.5
        gcmd.respond_info('[stepper_c]position_endstop:{0}'.format(self.adjust_c))
        str_xy = self.check_c_move.split(",") 
        buf_x = float(str_xy[0])
        buf_y = float(str_xy[1])
        pos = self.toolhead.get_position()
        old_val = 180.0
        cnt = 1
        pos[0] = buf_x
        pos[1] = buf_y
        pos[2] = pos[2]
        pos[3] = 0.0
        pos[4] = 0.0
        pos[5] = old_val
        self.toolhead.manual_move(pos, self.check_move_speed / 10.0)
        ret1 = self.probe.run_probe(gcmd)
        pos[0] = -buf_x
        pos[2] = ret1[2] + self.check_move_safe
        self.toolhead.manual_move(pos, self.check_move_speed)
        ret2 = self.probe.run_probe(gcmd)
        pos[2] = ret2[2] + self.check_move_safe
        self.toolhead.manual_move(pos, self.check_move_speed)
        delta = ret1[2] - ret2[2]
        new_val = -math.atan2(delta, buf_x * 2.0) / math.pi * 180.0 * kp + old_val
        gcmd.respond_info('{0}: {1} / {2} => new_val:{3} , delta:{4}'.format(
            cnt, ret1[2], ret2[2], new_val, delta))
        while(abs(delta) > 0.01):
            old_val = new_val
            cnt = cnt + 1
            pos[0] = buf_x
            pos[2] = ret2[2] + self.check_move_safe
            pos[5] = new_val
            self.toolhead.manual_move(pos, self.check_move_speed)
            ret1 = self.probe.run_probe(gcmd)
            pos[0] = -buf_x
            pos[2] = ret1[2] + self.check_move_safe
            self.toolhead.manual_move(pos, self.check_move_speed)
            ret2 = self.probe.run_probe(gcmd)
            pos[2] = ret2[2] + self.check_move_safe
            self.toolhead.manual_move(pos, self.check_move_speed)
            delta = ret1[2] - ret2[2]
            new_val = -math.atan2(delta, buf_x * 2.0) / math.pi * 180.0 * kp + old_val
            gcmd.respond_info('{0}: {1} / {2} => new_val:{3} , delta:{4}'.format(
                cnt, ret1[2], ret2[2], new_val, delta))
        gcmd.respond_info('Change position_endstop in [stepper_c] to {0}.'.format(
            self.adjust_c - old_val + 180.0)) 
        self.now_calibrate = False
            
    cmd_X_CALIBRATE_help = "X axis calibration for coreXYBC"
    def cmd_X_CALIBRATE(self, gcmd):
        self.now_calibrate = True
        gcmd.respond_info('[stepper_x]position_endstop:{0}'.format(self.adjust_x))
        str_prm = self.check_x_move.split(",")
        buf_cnt = int(str_prm[0])
        buf_xlow = float(str_prm[1])
        buf_xhigh = float(str_prm[2])
        buf_y = float(str_prm[3])
        pos = self.toolhead.get_position()
        fn_x = []
        fn_y = []
        for i in range(buf_cnt):
            pos[0] = (buf_xhigh - buf_xlow) / (buf_cnt - 1) * i + buf_xlow
            pos[1] = buf_y
            pos[2] = pos[2]
            pos[3] = 0.0
            pos[4] = 0.0
            pos[5] = 90.0
            self.toolhead.manual_move(pos, self.check_move_speed / 10.0)
            buf_ret = self.probe.run_probe(gcmd)
            pos[2] = buf_ret[2] + self.check_move_safe
            self.toolhead.manual_move(pos, self.check_move_speed)
            fn_x.append(pos[0])
            fn_y.append(buf_ret[2])
        a_low, b_low = linear_fit(fn_x, fn_y)
        str_out = ''
        for i in range(buf_cnt):
            str_out = str_out + '({0},{1}),'.format(fn_x[i], fn_y[i])
        gcmd.respond_info('xlow:a{0},b{1} by {2}'.format(a_low, b_low, str_out))
        fn_x = []
        fn_y = []
        for i in range(buf_cnt):
            pos[0] = (buf_xhigh - buf_xlow) / (buf_cnt - 1) * i + buf_xlow
            pos[1] = buf_y
            pos[2] = pos[2]
            pos[3] = 0.0
            pos[4] = 0.0
            pos[5] = -90.0
            self.toolhead.manual_move(pos, self.check_move_speed / 10.0)
            buf_ret = self.probe.run_probe(gcmd)
            pos[2] = buf_ret[2] + self.check_move_safe
            self.toolhead.manual_move(pos, self.check_move_speed)
            fn_x.append(pos[0])
            fn_y.append(buf_ret[2])
        a_high, b_high = linear_fit(fn_x, fn_y)
        str_out = ''
        for i in range(buf_cnt):
            str_out = str_out + '({0},{1}),'.format(fn_x[i], fn_y[i])
        gcmd.respond_info('xhigh:a{0},b{1} by {2}'.format(a_high, b_high, str_out))
        center_x = -(b_high - b_low) / (a_high - a_low) - self.probe_x_offset
        gcmd.respond_info('Change position_endstop and position_max in [stepper_x] to {0}.'.format(
            self.adjust_x - center_x))
        self.now_calibrate = False
            
    cmd_Y_CALIBRATE_help = "Y axis calibration for coreXYBC"
    def cmd_Y_CALIBRATE(self, gcmd):
        self.now_calibrate = True
        gcmd.respond_info('[stepper_y]position_endstop:{0}'.format(self.adjust_y))
        str_prm = self.check_y_move.split(",")
        buf_cnt = int(str_prm[0])
        buf_ylow = float(str_prm[1])
        buf_yhigh = float(str_prm[2])
        pos = self.toolhead.get_position()
        fn_x = []
        fn_y = []
        for i in range(buf_cnt):
            pos[0] = 0.0
            pos[1] = (buf_yhigh - buf_ylow) / (buf_cnt - 1) * i + buf_ylow
            pos[2] = pos[2]
            pos[3] = 0.0
            pos[4] = 0.0
            pos[5] = 0.0
            self.toolhead.manual_move(pos, self.check_move_speed / 10.0)
            buf_ret = self.probe.run_probe(gcmd)
            pos[2] = buf_ret[2] + self.check_move_safe
            self.toolhead.manual_move(pos, self.check_move_speed)
            fn_x.append(pos[1])
            fn_y.append(buf_ret[2])
        a_low, b_low = linear_fit(fn_x, fn_y)
        str_out = ''
        for i in range(buf_cnt):
            str_out = str_out + '({0},{1}),'.format(fn_x[i], fn_y[i])
        gcmd.respond_info('ylow:a{0},b{1} by {2}'.format(a_low, b_low, str_out))
        fn_x = []
        fn_y = []
        for i in range(buf_cnt):
            pos[0] = 0.0
            pos[1] = (buf_yhigh - buf_ylow) / (buf_cnt - 1) * i + buf_ylow
            pos[2] = pos[2]
            pos[3] = 0.0
            pos[4] = 0.0
            pos[5] = 180.0
            self.toolhead.manual_move(pos, self.check_move_speed / 10.0)
            buf_ret = self.probe.run_probe(gcmd)
            pos[2] = buf_ret[2] + self.check_move_safe
            self.toolhead.manual_move(pos, self.check_move_speed)
            fn_x.append(pos[1])
            fn_y.append(buf_ret[2])
        a_high, b_high = linear_fit(fn_x, fn_y)
        str_out = ''
        for i in range(buf_cnt):
            str_out = str_out + '({0},{1}),'.format(fn_x[i], fn_y[i])
        gcmd.respond_info('yhigh:a{0},b{1} by {2}'.format(a_high, b_high, str_out))
        center_y = -(b_high - b_low) / (a_high - a_low) - self.probe_y_offset
        gcmd.respond_info('Change position_endstop and position_max in [stepper_x] to {0}.'.format(
            self.adjust_y - center_y))
        self.now_calibrate = False

def load_kinematics(toolhead, config):
    return CoreXYBCKinematics(toolhead, config)
