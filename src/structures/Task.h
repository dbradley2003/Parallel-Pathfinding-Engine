#pragma once

#include "../structures/Position.h"

struct Task
{
	Task(Position _at, Direction _dir, unsigned int _flag)
	{
		at = _at;
		dir = _dir;
		flag = _flag;
	}

	Task()
	{

	}

	Position at;
	Direction dir;
	unsigned int flag;
};
