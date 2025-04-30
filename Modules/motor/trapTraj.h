/*
	Copyright 2021 codenocold 1107795287@qq.com
	Address : https://github.com/codenocold/dgm
	This file is part of the dgm firmware.
	The dgm firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
	The dgm firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _TRAP_TRAJ_H
#define _TRAP_TRAJ_H

#include "main.h"
#include <stdbool.h>

typedef struct sTraj
{
	// Step
	float Y;
	float Yd;
	float Ydd;

	float Tf_;

	float t;
	bool trajectory_done;
} tTraj;

extern tTraj Traj;

void TRAJ_plan(float Xf, float Xi, float Vi, float Vmax, float Amax, float Dmax);
void TRAJ_eval(float t);

#endif
