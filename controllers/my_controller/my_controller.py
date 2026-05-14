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
    OBSTACLE_THRESHOLD = 82
    SIDE_THRESHOLD = 80
    # max iterations to search for line before falling back
    MAX_SEARCH_CYCLES = 800
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
    # search timeout counter to avoid infinite circling when line lost
    search_counter = 0

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
    
        # print("Initializing sensors:", gs_vals)
    
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
            # print("Startup boundary ignore, count:", is_startup)
            # print(boundary_detected)

        # reset search counter when not actively searching
        if state != "SEARCH_LINE":
            search_counter = 0

        # print("GS:", gs_vals)
        any_on_black = any(v < LINE_THRESHOLD for v in gs_vals)

        # front sensors
        front_right = max(ps_vals[0], ps_vals[1])
        front_left = max(ps_vals[6], ps_vals[7])

        # side sensors
        side_right = ps_vals[2]
        side_left = ps_vals[5]
        # print("Side sensors:", "left:", side_left, "right:", side_right)
        # obstacle detection
        obstacle_front = (
            front_right > OBSTACLE_THRESHOLD or
            front_left > OBSTACLE_THRESHOLD
        )
        # print("Obstacle front:", obstacle_front)

        # =====================================================
        # FOLLOW LINE
        # =====================================================
        if boundary_detected and startup_ignore <= 0:
            is_startup = 10
            print(boundary_detected)
            # print("Startup count:", is_startup)
            print("🛑 Boundary detected!")
            # STEP 1: reverse strongly
            left_motor.setVelocity(-4.0)
            right_motor.setVelocity(-4.0)

            robot.step(5000)


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
                print("Turn direction:", "right" if turn_dir == 1 else "left")
                direction = "right" if turn_dir == 1 else "left"
                state = "STOP"
                counter = 0

                print("🚨 Obstacle detected!")

            else:
                
                if any_on_black:

                    error = gs_vals[2] - gs_vals[0]
                    counter = 0
                    
                    is_centered = gs_vals[1] < 450 and gs_vals[0] > 650 and gs_vals[2] > 650
                    

                    if line_reacquire_slow > 0:
                        current_speed = 0.4  # Move very slowly to allow for rotation
                        # Increase Kp sensitivity during alignment to turn sharper
                        correction = (Kp * 2.0) * error + Kd * (error - last_error)
                        
                        if is_centered:
                            line_reacquire_slow = 0 # Exit slow mode early if we are straight
                            print("🎯 Aligned! Speeding up.")
                        else:
                            line_reacquire_slow -= 1
                    else:
                        current_speed = BASE_SPEED
                        correction = Kp * error + Kd * (error - last_error)
                    
                    l_speed = current_speed - correction
                    r_speed = current_speed + correction
                    last_error = error
                
                else: 
                    counter += 1
                    
                    if counter < 20:
                        error = 250 if last_error > 0 else -250
                        
                    elif counter < 80:
                        l_speed = 2.5
                        r_speed = 2.5
                        if last_error > 0:
                            l_speed += 0.7
                        else:
                            r_speed += 0.7
                        left_motor.setVelocity(l_speed)
                        right_motor.setVelocity(r_speed)
                        continue
                        
                    else:
                        print("⚠️ Line completely lost! Initiating spin search...")
                        # Pass the last known direction so it spins the right way
                        turn_dir = 1 if last_error > 0 else -1 
                        state = "SEARCH_LINE" # Jump to your dedicated search state
                        counter = 0
                        continue
                d_error = error - last_error
                last_error = error

                correction = Kp * error + Kd * d_error

                current_speed = BASE_SPEED

                # slow temporarily after finding line again
                if line_reacquire_slow > 0:
                
                    current_speed = 0.3
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
                l_speed, r_speed = -6.0, 6.0
                # print("Turning right to avoid obstacle")
            else:
                l_speed, r_speed = 4.0, -4.0
                # print("Turning left to avoid obstacle")

            counter += 1

            if counter > 30:
                state = "WALL_FOLLOW"
                counter = 0
                wall_lost = 0


                # print("🧱 Moving around obstacle")

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
                # print('everher')
                wall_here = (
                    side_right > SIDE_THRESHOLD
                    if turn_dir == 1
                    else side_left > SIDE_THRESHOLD
                )
                # print("Wall here:", wall_here)
                if direction == "right":
                    # print("acrully right")
                    l_speed, r_speed = 3.5, 3.0
                    wall_lost  +=1

                else:
                    # print("actually left")
                    l_speed, r_speed = 3.0, 3.5
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
        # SEARCH LINE
        # =====================================================
        elif state == "SEARCH_LINE":
            
            if obstacle_front:
                # Decide which way to turn based on the sensors
                turn_dir = -1 if front_left > front_right else 1
                
                # Restart the obstacle avoidance process
                state = "STOP"
                counter = 0
                print("New obstacle detected while searching!")
            
            else:

                search_counter += 1

                if turn_dir == 1:
                    l_speed, r_speed = 3.5, 3.0
                else:
                    l_speed, r_speed = 3.0, 3.5

                if search_counter > MAX_SEARCH_CYCLES:
                    print("Search timeout reached — switching strategy")
                    turn_dir = -turn_dir
                    state = "TURN_AWAY"
                    counter = 0
                    cooldown = 50
                    search_counter = 0
                    l_speed, r_speed = 0.0, 0.0

                if any_on_black:
                    state = "FOLLOW_LINE"
                    line_reacquire_slow = 500
                    last_error = 0
                    search_counter = 0
                    print(" Line found! Entering precision alignment...")
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