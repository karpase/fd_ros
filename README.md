# fd_ros
ROS Interface for using Fast Downward without Translating

This file contains a slightly modified version of Fast Downward. It reads the translated input file (the FD translator should be run first), and then publishes a service which gets a complete initial state and returns a plan.

To compile, run:
```
catkin build -DFORCE_DYNAMIC_BUILD=YES -DUSE_LP=False
```

To run the planner:

First, make sure roscore is running somewhere).
Then, in a shell run

```
translate.py <domain.pddl> <problem.pddl>  #This is not included here
rosrun fd_ros downward --search "astar(lmcut())" < output.sas   
```

And in another shell (the client) run:
```
rosservice call /fd_plan "init_state_facts: [{name: 'on', args: ['a','b']}, {name: 'on', args: ['b','c']}, {name: 'on', args: ['c','d']}, {name: 'ontable', args: ['d']}, {name: 'clear', args: ['a']}, {name: 'handempty', args: []}]
```

You should get:
```
unsolvable: False
solved: True
plan: ['unstack a b', 'put-down a', 'unstack b c', 'stack b a', 'unstack c d', 'stack c b', 'pick-up d', 'stack d c']
```

TODOs:
* improve efficiency of pddl fact lookup
* handle unsolvable
* return proper error when initial state is incomplete
* so much more
