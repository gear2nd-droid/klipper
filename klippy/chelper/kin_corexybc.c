// CoreXY kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord

/*
static double
corexy_stepper_plus_calc_position(struct stepper_kinematics *sk, struct move *m
                                  , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    return c.x + c.y;
}

static double
corexy_stepper_minus_calc_position(struct stepper_kinematics *sk, struct move *m
                                   , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    return c.x - c.y;
}

struct stepper_kinematics * __visible
corexy_stepper_alloc(char type)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    if (type == '+')
        sk->calc_position_cb = corexy_stepper_plus_calc_position;
    else if (type == '-')
        sk->calc_position_cb = corexy_stepper_minus_calc_position;
    sk->active_flags = AF_X | AF_Y;
    return sk;
}*/

struct corexybc_stepper {
    struct stepper_kinematics sk;
	char axis;
	double adjust_x;
	double adjust_y;
	double adjust_z;
	double adjust_a;
	double adjust_b;
	double adjust_c;
    double prev;
};

static double
corexybc_stepper_calc_position(struct stepper_kinematics *sk, struct move *m
                            , double move_time)
{
    struct corexybc_stepper *ds = container_of(sk, struct corexybc_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
	
	double pos_x, pos_y, pos_z, pos_b, pos_c;
	pos_x = c.x;
	pos_y = c.y;
	pos_z = c.z;
	pos_b = c.b;
	pos_c = c.c;
	double buf_x = pos_x;
	double buf_y = pos_y * cos(ds->adjust_a / 180.0 * M_PI) - pos_z * sin(ds->adjust_a / 180.0 * M_PI);
	double buf_z = pos_y * sin(ds->adjust_a / 180.0 * M_PI) + pos_z * cos(ds->adjust_a / 180.0 * M_PI);
	
	double pos = 0.0;
    if (ds->axis == 'x') {
		//pos = pos_x;
		//pos = pos_x + pos_y;
		pos = buf_x + buf_y;
        ds->prev = pos;
    } else if (ds->axis == 'y') {
		//pos = pos_y;
		//pos = pos_x - pos_y;
		pos = buf_x - buf_y;
        ds->prev = pos;
    } else if (ds->axis == 'z') {
		//pos = pos_z;
		//pos = pos_z + pos_y * tan(ds->adjust_a / 180.0 * M_PI);
		pos = buf_z;
        ds->prev = pos;
    } else if (ds->axis == 'b') {
		pos = pos_b;
        ds->prev = pos;
    } else if (ds->axis == 'c') {
		pos = pos_c;
        ds->prev = pos;
    }
    return pos;
}

struct stepper_kinematics * __visible
corexybc_stepper_alloc(char axis, 
						double adjust_x, double adjust_y, double adjust_z, 
						double adjust_a, double adjust_b, double adjust_c)
{
    struct corexybc_stepper *ds = malloc(sizeof(*ds));
    memset(ds, 0, sizeof(*ds));
	ds->axis = axis;
	ds->adjust_x = adjust_x;
	ds->adjust_y = adjust_y;
	ds->adjust_z = adjust_z;
	ds->adjust_a = adjust_a;
	ds->adjust_b = adjust_b;
	ds->adjust_c = adjust_c;
	ds->prev = 0.0;
	ds->sk.calc_position_cb = corexybc_stepper_calc_position;
    ds->sk.active_flags = AF_X | AF_Y | AF_Z | AF_A | AF_B | AF_C;
    return &ds->sk;
}
