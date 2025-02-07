// Trunnion BC kinematics stepper pulse time generation
//
// Copyright (C) 2023-  @_geek_gear_ <gear2nd.droid@gmail.com>
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

struct trunnion_bc_stepper {
    struct stepper_kinematics sk;
	char axis;
	double offset_x;
	double offset_y;
	double offset_z;
	double offset_a;
	double offset_b;
	double offset_c;
    double prev;
};

static double
trunnion_bc_stepper_calc_position(struct stepper_kinematics *sk, struct move *m
                            , double move_time)
{
    struct trunnion_bc_stepper *ds = container_of(sk, struct trunnion_bc_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
	
	double pos_x, pos_y, pos_z, pos_b, pos_c;
	pos_x = c.x;
	pos_y = c.y * cos(ds->offset_a) - c.z * sin(ds->offset_a);
	pos_z = c.y * sin(ds->offset_a) + c.z * cos(ds->offset_a);
	pos_b = c.b;
	pos_c = c.c;
	
	double pos = 0.0;
    if (ds->axis == 'x') {
		pos = pos_x;
        ds->prev = pos;
    } else if (ds->axis == 'y') {
		pos = pos_y;
        ds->prev = pos;
    } else if (ds->axis == 'z') {
		pos = pos_z;
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
trunnion_bc_stepper_alloc(char axis, 
						double offset_x, double offset_y, double offset_z, 
						double offset_a, double offset_b, double offset_c)
{
    struct trunnion_bc_stepper *ds = malloc(sizeof(*ds));
    memset(ds, 0, sizeof(*ds));
	ds->axis = axis;
	ds->offset_x = offset_x;
	ds->offset_y = offset_y;
	ds->offset_z = offset_z;
	ds->offset_a = offset_a;
	ds->offset_b = offset_b;
	ds->offset_c = offset_c;
	ds->prev = 0.0;
	ds->sk.calc_position_cb = trunnion_bc_stepper_calc_position;
    ds->sk.active_flags = AF_X | AF_Y | AF_Z | AF_A | AF_B | AF_C;
    return &ds->sk;
}
