import sys
import time
from fsm import Fsm
from nao_soccer import NaoSoccer


# functions (actions of the FSM)
def do_ball_search():
    if nao.ball_found():
        return 'ball_found'
    else:
        nao.search_ball()
        return 'ball_not_found'


def do_ball_track():
    # TODO : add a 'looking at the ball' condition
    width, height = nao.get_image_dimension()
    ball_radius = nao.get_ball_radius()

    target_x = width / 2  # center the ball horizontally
    target_y = height - 3 * ball_radius  # bottom the ball

    nao.track_ball(target_x, target_y)

    return 'not_looking_at_the_ball' or 'looking_at_the_ball'


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
            print("New Event : ", next_event)
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
