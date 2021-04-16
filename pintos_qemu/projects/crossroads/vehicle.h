#ifndef __PROJECTS_PROJECT1_VEHICLE_H__
#define __PROJECTS_PROJECT1_VEHICLE_H__

#include "projects/crossroads/position.h"

#define VEHICLE_STATUS_READY 	0
#define VEHICLE_STATUS_RUNNING	1
#define VEHICLE_STATUS_FINISHED	2

struct vehicle_info {
	char id;
	char state;
	char start;
	char dest;
	struct position position;
	struct lock **map_locks;
};

void vehicle_loop(void *vi);
int try_move(int start, int dest, int step, struct vehicle_info *vi);
void release_blocked_threads(void **_maplock);
#endif /* __PROJECTS_PROJECT1_VEHICLE_H__ */