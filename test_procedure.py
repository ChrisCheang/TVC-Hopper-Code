def test_procedure(odrv0):
    print("entered test procedure")
    START_POS_R2 = 0
    START_POS_D2 = 0
    CPR = 8192

    odrv0.axis0.motor.config.current_lim = 20
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.input_filter_bandwidth = 2 # Set the filter bandwidth [1/s]
    odrv0.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER # Activate the setpoint filter
    
    odrv0.axis1.motor.config.current_lim = 20
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.input_filter_bandwidth = 2 # Set the filter bandwidth [1/s]
    odrv0.axis1.controller.config.input_mode = INPUT_MODE_POS_FILTER # Activate the setpoint filter


    #move to 14 up
    odrv0.axis0.controller.input_pos=7.1774
    odrv0.axis1.controller.input_pos=7.1774
    time.sleep(2)

    odrv0.axis0.controller.config.input_filter_bandwidth = 9 # Increase filter bandwidth for circles
    odrv0.axis1.controller.config.input_filter_bandwidth = 9 # Increase filter bandwidth for circles


    #14 deg circle - starts at 14 up, circle, ends at 14 up (65 waypoints)
    waypoints0 = [-7.1774,-6.4577,-5.6811,-4.8557,-3.9901,-3.0931,-2.1735,-1.2403,-0.3026,0.6309,1.5512,2.4500,3.3189,4.1500,4.9358,5.6692,6.3436,6.9531,7.4922,7.9561,8.3410,8.6434,8.8608,8.9915,9.0344,8.9893,8.8568,8.6380,8.3350,7.9505,7.4876,6.9505,6.3436,5.6722,4.9421,4.1595,3.3312,2.4646,1.5674,0.6478,-0.2858,-1.2245,-2.1594,-3.0811,-3.9807,-4.8489,-5.6768,-6.4558,-7.1774,-7.8341,-8.4186,-8.9245,-9.3462,-9.6791,-9.9194,-10.0645,-10.1127,-10.0635,-9.9176,-9.6766,-9.3433,-8.9216,-8.4161,-7.8326,-7.1774]
    waypoints1 = [-7.1774,-7.8326,-8.4161,-8.9216,-9.3433,-9.6766,-9.9176,-10.0635,-10.1127,-10.0645,-9.9194,-9.6791,-9.3462,-8.9245,-8.4185,-7.8341,-7.1774,-6.4557,-5.6768,-4.8489,-3.9807,-3.0811,-2.1593,-1.2245,-0.2858,0.6478,1.5674,2.4646,3.3312,4.1595,4.9421,5.6723,6.3436,6.9505,7.4876,7.9505,8.3350,8.6380,8.8568,8.9893,9.0344,8.9915,8.8608,8.6434,8.3410,7.9561,7.4922,6.9531,6.3436,5.6692,4.9358,4.1500,3.3189,2.4500,1.5512,0.6308,-0.3026,-1.2403,-2.1735,-3.0931,-3.9902,-4.8557,-5.6811,-6.4577,-7.1774]

    waypoints = [[-waypoints0[i],-waypoints1[i]] for i in range(len(waypoints0))]

    for i in waypoints:
        odrv0.axis0.controller.input_pos=i[0]
        odrv0.axis1.controller.input_pos=i[1]

        check_state(odrv0)
        if state != 3:
            state_machine(odrv0)
        time.sleep(0.04)

    #move to 10 deg up to prepare for fast circle
    odrv0.axis0.controller.config.input_filter_bandwidth = 5 # Decrease filter bandwidth to move to 10 deg up
    odrv0.axis1.controller.config.input_filter_bandwidth = 5 # Decrease filter bandwidth to move to 10 deg up
    odrv0.axis0.controller.input_pos = 5.06  #10 deg up
    odrv0.axis1.controller.input_pos = 5.06
    time.sleep(0.25)

    odrv0.axis0.controller.config.input_filter_bandwidth = 10 # Increase filter bandwidth for circles
    odrv0.axis1.controller.config.input_filter_bandwidth = 10 # Increase filter bandwidth for circles

    #10 deg circle - starts at 10 up, fast circle, ends at 10 up (65 waypoints)
    waypoints0 = [-5.0552,-4.5449,-3.9934,-3.4064,-2.7899,-2.1500,-1.4930,-0.8254,-0.1536,0.5159,1.1767,1.8228,2.4479,3.0464,3.6128,4.1417,4.6285,5.0687,5.4583,5.7940,6.0726,6.2917,6.4495,6.5446,6.5761,6.5438,6.4481,6.2899,6.0706,5.7920,5.4568,5.0678,4.6285,4.1428,3.6150,3.0498,2.4523,1.8280,1.1825,0.5220,-0.1475,-0.8196,-1.4878,-2.1455,-2.7863,-3.4038,-3.9917,-4.5441,-5.0552,-5.5197,-5.9328,-6.2900,-6.5875,-6.8223,-6.9917,-7.0940,-7.1281,-7.0936,-6.9909,-6.8212,-6.5863,-6.2887,-5.9317,-5.5191,-5.0552]
    waypoints1 = [-5.0552,-5.5191,-5.9317,-6.2887,-6.5863,-6.8212,-6.9909,-7.0936,-7.1281,-7.0940,-6.9917,-6.8223,-6.5875,-6.2899,-5.9327,-5.5197,-5.0552,-4.5441,-3.9917,-3.4038,-2.7863,-2.1455,-1.4878,-0.8196,-0.1475,0.5220,1.1825,1.8280,2.4523,3.0498,3.6150,4.1428,4.6285,5.0678,5.4568,5.7920,6.0706,6.2899,6.4481,6.5438,6.5761,6.5445,6.4495,6.2917,6.0726,5.7940,5.4583,5.0687,4.6285,4.1417,3.6128,3.0464,2.4479,1.8228,1.1767,0.5158,-0.1537,-0.8254,-1.4930,-2.1500,-2.7899,-3.4064,-3.9934,-4.5449,-5.0552]

    waypoints = [[-waypoints0[-i],-waypoints1[-i]] for i in range(len(waypoints0))]  #-i to flip turn direction

    for i in waypoints:
        odrv0.axis0.controller.input_pos=i[0]
        odrv0.axis1.controller.input_pos=i[1]

        check_state(odrv0)
        if state != 3:
            state_machine(odrv0)
        time.sleep(0.01)

    #hold at 10 up for a short bit after circle
    odrv0.axis0.controller.input_pos = 5.06  #10 deg up
    odrv0.axis1.controller.input_pos = 5.06
    time.sleep(0.3)

    #reduce bandwidth and return to neutral
    odrv0.axis0.controller.config.input_filter_bandwidth = 3 # Decrease filter bandwidth to return to neutral
    odrv0.axis1.controller.config.input_filter_bandwidth = 3 # Decrease filter bandwidth to return to neutral
    odrv0.axis0.controller.input_pos = 0
    odrv0.axis1.controller.input_pos = 0

    while True:
        check_state(odrv0)
        if state != 3:
            state_machine(odrv0)
        time.sleep(t_sleep)
