# Overview
To ensure reliable communication between modules, we need to agree on common message formats and naming conventions.
The definitions in the proposal are a good start, but I thought I'd clarify them a bit and make a few extensions.
Everything below is just a proposal, so feel free to make changes as long as we're all on the same page.

# Planner Input
```
{
	"rooms": [ // only known rooms
		{
			"id": "2,4",
			"robots": [
				"robot1",
				...
			],
			"switches": [
				"S1",
				...
			],
			"doors": [
				{
					"id": "D1",
					"other_room": "2,3" // or "UNKNOWN" if unexplored
				},
				...
			]
			"exit": false
		},
		...
	],
	"robots": [ // contains all robots
		{
			"id": "robot1",
			"colors": [
				"red",
				"blue",
				...
			],
			"status": {
                "type": "ok", // or "broken" or "waiting"
                "repair_time": null, // int, number of seconds until the robot starts working again
                "waiting_for": null, // 
            },
			"task": "EXPLORE:2,4" // standard task name or null if no task
		},
        {
			"id": "robot2",
			"colors": [
				"red",
				...
			],
			"status": {
                "type": "waiting",
                "repair_time": null,
                "waiting_for": [ // this robot can't complete its task until other robots complete these tasks
                    "HOLD:S1",
                    ...
                ]
            },
			"task": "CROSS:D1"
		},
		...
	],
	"switches": [ // only known switches
		{
			"id": "S1",
			"color": "red",
			"mode": "hold", // or "latch" if it only needs to be triggered once
			"active": false,
			"controls": "D1"
		},
		...
	],
	"doors": [ // only known doors
		{
			"id": "D1",
			"open": false,
			"open_if": {
				"any_of": [ // or "all_of"
					"S1",
					"S2",
					...
				]
			}
		},
		...
	],
	"tasks" [ // tasks that are currently in progress
		{
			"id": "HOLD:S1",
			"type": "HOLD",
			"target": "S1",
			"needs_color": "red", // only used for ACTIVATE and HOLD
			"enables": "CROSS:D1", // only used for ACTIVATE and HOLD, this task should be canceled once the enabled task is complete
			"depends": null
		},
		{
			"id": "CROSS:D1",
			"type": "CROSS",
			"target": "D1",
			"needs_color": null,
			"enables": null,
			"depends": [ // only used for CROSS, these tasks must continue until this one is done
				"HOLD:S1",
				...
			]
		},
		{
			"id": "EXPLORE:2,4",
			"type": "EXPLORE",
			"target": "2,4",
			"needs_color": null,
			"enables": null,
			"depends": null
		},
		{
			"id": "EXIT",
			"type": "EXIT",
			"target": "1,3",
			"needs_color": null,
			"enables": null,
			"depends": null
		},
		...
	],
	"costs": { // Costs are path lengths on the known map. A missing entry means the target is unreachable
		"robot1": {
			"HOLD:S1": 2.0,
			"CROSS:D1": 5.0,
			...
		},
		...
	}
}
```

# Planner Output
```
// all robots must have orders for each replanning loop, use WAIT for idle robots
{
	"robot1": "HOLD:S1",
	...
}
```

# Naming Conventions
* Doors are assigned numbers in order from W-E and N-S, so D1 would be the door in the NW corner of the map.
* Switches follow the same pattern: S1, S2, etc.
* Robots are numbered, e.g. robot1, robot2, etc.
* Tasks are named according to Table 1 in the proposal.
* Rooms are named based on a 2D grid, going W-E and N-S. 1,1 is the room in the NW corner, 1,2 is the room due east of it, and so on.
# Notes from the implementation
These follow from how the simulator reveals the maze; see the world-state PR (#4) for details.
* **Exploring:** a room becomes known as soon as a robot enters it, and everything in it is revealed at once. So exploring is `CROSS:<door>` for a door whose `other_room` is `"UNKNOWN"`, and no separate `EXPLORE` task is generated. A room nobody has entered can't be named without revealing it.
* **`open_if`:** it lists only switches the team has found. It is `null` while none of the door's switches has been found (and for a door that needs no switch); then use the door's `open` flag.
* **Robot `status` and `task`:** reported by the executor. When a robot finishes its task, its `task` goes back to `null`.
