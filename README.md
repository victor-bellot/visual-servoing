# UE52VSIK-BELL-ROUS

## The team

- BELLOT Victor (victor.bellot@ensta-bretagne.org)
- ROUSSEL Etienne (etienne.roussel@ensta-bretagne.org)


## Strategy

### Actions

- head's yaw & pitch rotation
- forward & lateral walking speed
- body yaw rotation

### Measurements

- ball_distance : 0 if not found, the distance otherwise
- goal_detected : boolean
- ball center pixel
- goal barycenter pixel

### Relations

- ball_distance -> dx & dy (to get ready for shooting)
- ball & goal position -> head & body atitude -> coupled by dy

### Using a Finite State Machine (FSM)

- described in 'fsm_nao_soccer.txt'
- run using 'run_fsm.py'
- NaoSoccer's methods in the NaoSoccer class

### Results

Un suspens de 90s : [video](nao_goal.mp4)