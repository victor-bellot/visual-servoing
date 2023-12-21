import sys
import time
from fsm import Fsm
from nao_soccer import NaoSoccer


# functions (actions of the FSM)
def do_start_over():
    time.sleep(1.0)
    return 'ready'


def do_ball_search():
    if nao.ball_found():
        return 'ball_found'
    else:
        nao.search_ball()
        return 'ball_not_found'


def do_ball_track():
    if nao.ball_found():
        if nao.ball_in_sight():
            return 'ball_tracked'
        else:
            nao.head_align_body()
            nao.track_ball()
            nao.move()
            return 'ball_not_tracked'
    else:
        return 'error'


def do_ball_reach():
    if nao.ball_found():
        if nao.ball_reached():
            return 'ball_reached'
        else:
            nao.head_align_body()
            nao.track_ball()
            nao.reach_ball()
            nao.move()
            return 'ball_not_reached'
    else:
        return 'error'


def do_goal_search():
    if nao.ball_found():
        if nao.goal_found():
            return 'goal_found'
        else:
            nao.head_align_body()
            nao.track_ball()
            nao.reach_ball()
            nao.search_goal()
            nao.move()
            return 'goal_not_found'
    else:
        return 'goal_not_found'  # 'error'


def do_shoot():
    if nao.goal_found():
        if nao.shoot_done():
            return 'shoot_done'
        else:
            nao.shoot()
            nao.move()
            return 'shoot_not_done'
    else:
        return 'goal_not_found'


def do_stop():
    return 'done'


if __name__ == "__main__":
    # Get the user's name (if available)
    user = sys.argv[1] if len(sys.argv) > 1 else 'nao'

    # Define desired frame rate
    fps = 10
    dt = 1. / fps

    # Set your ip HERE
    if user in 'etienne':
        robot_ip = "localhost"
    elif user in 'victor':
        robot_ip = "localhost"
    else:
        robot_ip = None

    # Set your Nao path HERE
    if user in 'etienne':
        virtual_camera_path = "/home/etrange/Documents/ensta/rob/Semestre5/visual_servoing/UE52-VS-IK/imgs"
    elif user in 'victor':
        virtual_camera_path = "/home/victor/nao/UE52-VS-IK/imgs"
    else:
        virtual_camera_path = None

    fsm = Fsm()  # finite state machine
    fsm.load_fsm_from_file("fsm_nao_soccer.txt")

    # Instantiate a NaoSoccer robot
    nao = NaoSoccer(robot_ip, virtual_camera_path)

    # Finite State Machine loop
    running = True
    while running:
        t_loop = time.time()
        nao.reset()  # reset the Nao Soccer before each action

        action = fsm.run()  # action to be executed in the new state
        next_event = action()  # new event when state action is finished

        if fsm.current_state != fsm.end_state:
            fsm.set_event(next_event)  # set new event for next transition
        else:
            running = False

        t_left = dt - (time.time() - t_loop)
        if t_left > 0:
            time.sleep(t_left)
        else:
            print("Out of time...")

    print("End of the program")
    exit()
