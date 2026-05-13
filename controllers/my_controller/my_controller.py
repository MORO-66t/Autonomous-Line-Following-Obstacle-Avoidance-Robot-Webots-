from controller import Robot

def run_robot():
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    # =========================
    # motors
    # =========================
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')

    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    # =========================
    # ground sensors
    # =========================
    gs = [robot.getDevice(f'gs{i}') for i in range(3)]

    for s in gs:
        s.enable(time_step)

    # =========================
    # distance sensors
    # =========================
    ps = [robot.getDevice(f'ps{i}') for i in range(8)]

    for s in ps:
        s.enable(time_step)

    # =========================
    # constants
    # =========================
    BASE_SPEED = 2.0

    Kp = 0.005
    Kd = 0.001

    LINE_THRESHOLD = 600

    # obstacle detection tuned
    OBSTACLE_THRESHOLD = 100
    SIDE_THRESHOLD = 200

    # =========================
    # states
    # =========================
    state = "FOLLOW_LINE"
    line_reacquire_slow = 0
    counter = 0
    last_error = 0
    turn_dir = 1
    wall_lost = 0
    startup_ignore = 10
    is_startup = 10

    # cooldown لمنع التكرار
    cooldown = 0
    l_speed = 0.0
    r_speed = 0.0
# =====================================
# sensor stabilization
# =====================================

    # wait until sensors stabilize
    while True:

        robot.step(time_step)
    
        gs_vals = [s.getValue() for s in gs]
    
        avg_ground = sum(gs_vals) / 3
    
        print("Initializing sensors:", gs_vals)
    
        # white floor should be high values
        if avg_ground > 600:
            break
    while robot.step(time_step) != -1:
        if startup_ignore > 0:
            startup_ignore -= 1

        # =========================
        # cooldown update
        # =========================
        if cooldown > 0:
            cooldown -= 1

        # =========================
        # read sensors
        # =========================
        gs_vals = [s.getValue() for s in gs]
        ps_vals = [s.getValue() for s in ps]

        avg_ground = sum(gs_vals) / 3

        boundary_detected = (
            gs_vals[0] < 320 and
            gs_vals[1] < 320 and
            gs_vals[2] < 320
        )
        sensor_return_normal = (
            gs_vals[0] > 500 or
            gs_vals[1] > 500 or
            gs_vals[2] > 500
        )
        if  sensor_return_normal:
            is_startup = 10
            # print("Sensors back to normal, reset startup ignore")
            # print(is_startup)
        if is_startup != 0 and boundary_detected:
            boundary_detected = False
            is_startup -= 1
            print("Startup boundary ignore, count:", is_startup)
            print(boundary_detected)

        print("GS:", gs_vals)
        any_on_black = any(v < LINE_THRESHOLD for v in gs_vals)

        # front sensors
        front_right = max(ps_vals[0], ps_vals[1])
        front_left = max(ps_vals[6], ps_vals[7])

        # side sensors
        side_right = max(ps_vals[1], ps_vals[2])
        side_left = max(ps_vals[5], ps_vals[6])

        # obstacle detection
        obstacle_front = (
            front_right > OBSTACLE_THRESHOLD or
            front_left > OBSTACLE_THRESHOLD
        )

        # =====================================================
        # FOLLOW LINE
        # =====================================================
        if boundary_detected and startup_ignore <= 0:
            is_startup = 10
            print(boundary_detected)
            print("Startup count:", is_startup)
            print("🛑 Boundary detected!")
            # STEP 1: reverse strongly
            left_motor.setVelocity(-4.0)
            right_motor.setVelocity(-4.0)

            robot.step(5000)

            # STEP 2: rotate AWAY harder
            # choose direction based on previous line side

            if gs_vals[0] < gs_vals[2]:
                # line more on left before losing
                left_motor.setVelocity(4.0)
                right_motor.setVelocity(-4.0)
            else:
                # line more on right
                left_motor.setVelocity(-4.0)
                right_motor.setVelocity(4.0)

            robot.step(700)

            # STEP 3: move forward slightly
            left_motor.setVelocity(3.0)
            right_motor.setVelocity(3.0)

            robot.step(400)

            continue
        if state == "FOLLOW_LINE":

            # detect obstacle only if cooldown finished
            if obstacle_front and cooldown == 0:

                turn_dir = -1 if front_left > front_right else 1

                state = "STOP"
                counter = 0

                print("🚨 Obstacle detected!")

            else:
                
                if any_on_black:
                                # entering line vertically / not aligned yet
                    if (
                        counter > 3 and
                        gs_vals[1] < 500 and
                        abs(gs_vals[0] - gs_vals[2]) < 120
                    ):
                        line_reacquire_slow = 40
                                    # line just found again
                    if counter > 0:
                        line_reacquire_slow = 40
                
                    counter = 0
                
                    error = gs_vals[2] - gs_vals[0]
                
                else:
                
                    counter += 1
                
                    # short line loss
                    if counter < 20:
                    
                        error = 250 if last_error > 0 else -250
                
                    else:
                    
                        # forward recovery movement
                        l_speed = 2.5
                        r_speed = 2.5
                
                        # slight curve memory
                        if last_error > 0:
                            l_speed += 0.7
                        else:
                            r_speed += 0.7
                
                        left_motor.setVelocity(l_speed)
                        right_motor.setVelocity(r_speed)
                
                        continue
                d_error = error - last_error
                last_error = error

                correction = Kp * error + Kd * d_error

                current_speed = BASE_SPEED

                # slow temporarily after finding line again
                if line_reacquire_slow > 0:
                
                    current_speed = 0.10
                    line_reacquire_slow -= 1
                
                l_speed = current_speed - correction
                r_speed = current_speed + correction

        # =====================================================
        # STOP
        # =====================================================
        elif state == "STOP":

            l_speed, r_speed = 0.0, 0.0

            counter += 1

            if counter > 5:
                state = "TURN_AWAY"
                counter = 0

        # =====================================================
        # TURN AWAY
        # =====================================================
        elif state == "TURN_AWAY":

            if turn_dir == 1:
                l_speed, r_speed = -4.0, 4.0
            else:
                l_speed, r_speed = 4.0, -4.0

            counter += 1

            if counter > 20:
                state = "WALL_FOLLOW"
                counter = 0
                wall_lost = 0


                print("🧱 Moving around obstacle")

        # =====================================================
        # WALL FOLLOW
        # =====================================================
        elif state == "WALL_FOLLOW":

            if obstacle_front:

                if turn_dir == 1:
                    l_speed, r_speed = -2.0, 2.0
                else:
                    l_speed, r_speed = 2.0, -2.0

                wall_lost = 0

            else:

                wall_here = (
                    side_right > SIDE_THRESHOLD
                    if turn_dir == 1
                    else side_left > SIDE_THRESHOLD
                )

                if wall_here:
                    l_speed, r_speed = 3.00, 3.5
                    wall_lost = 0

                else:
                    l_speed, r_speed = 3.00, 3.5
                    wall_lost += 1

            counter += 1

            if wall_lost > 25 and counter > 40:
                state = "TURN_BACK"
                counter = 0

        # =====================================================
        # TURN BACK
        # =====================================================
        elif state == "TURN_BACK":

            if turn_dir == 1:
                l_speed, r_speed = 3.0, -3.0
            else:
                l_speed, r_speed = -3.0, 3.0

            counter += 1

            if counter > 12:
                state = "SEARCH_LINE"
                counter = 0

                print("🔍 Searching for line")

        # =====================================================
        # =====================================================
        # SEARCH LINE
        # =====================================================
        elif state == "SEARCH_LINE":

            if turn_dir == 1:
                l_speed, r_speed = 3.5, 3.0
            else:
                l_speed, r_speed = 3.0, 3.5
        
            # line found again
            if any_on_black and not obstacle_front:
            
                state = "FOLLOW_LINE"
                last_error = 0
        
                # start cooldown
                cooldown = 40
        
                print("✅ Back on line")
        # =========================
        # speed limits
        # =========================
        l_speed = max(-6.28, min(6.28, l_speed))
        r_speed = max(-6.28, min(6.28, r_speed))

        # apply speed
        left_motor.setVelocity(l_speed)
        right_motor.setVelocity(r_speed)

if __name__ == "__main__":
    run_robot()