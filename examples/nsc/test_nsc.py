# -*- coding: utf-8 -*-

from time import sleep
from dronekit import connect, VehicleMode
from dronekit_sitl import SITL
from drone_trees.control_automaton import ControlAutomaton
from drone_trees.drone_tree_vehicle import DroneTreeVehicle
from fly_nsc import behaviour_tree


def start_sitl():
    """Launch a SITL using local copy of Copter 4,
    then set up the simulator for a rangefinder"""
    sitl = SITL('sitl/ArduCopter.exe')
    sitl.launch(['--home=51.454531,-2.629158,589,353'])

    veh = connect(sitl.connection_string(), vehicle_class=DroneTreeVehicle)

    veh.parameters['SIM_SONAR_SCALE'] = 0.00001
    veh.parameters['RNGFND2_ORIENT'] = 0
    veh.parameters['RNGFND2_SCALING'] = 10
    veh.parameters['RNGFND2_PIN'] = 0
    veh.parameters['RNGFND2_TYPE'] = 1
    veh.parameters['RNGFND2_MAX_CM'] = 5000
    veh.parameters['RNGFND2_MIN_CM'] = 5000

    veh.parameters['SIM_BATT_VOLTAGE'] = 12.59
    veh.parameters['BATT_MONITOR'] = 4

    veh.parameters['TERRAIN_ENABLE'] = 0

    veh.parameters['ARMING_CHECK'] = 16384  # mission only
    veh.parameters['FRAME_CLASS'] = 2       # I'm a hex

    #veh.parameters['FS_OPTION'] = 3
    veh.parameters['FS_THR_ENABLE'] = 2
    veh.parameters['FS_GCS_ENABLE'] = 2

    veh.close()

    sitl.stop()
    sitl.launch(['--home=51.454531,-2.629158,589,353'],
                use_saved_data=True,
                verbose=True)
    sitl.block_until_ready()

    return sitl


def test_sitl():
    """Test that the SITL launch function start_sitl works
    and that we can use it to access distance_sensors attributes"""
    sitl = start_sitl()
    veh = connect(sitl.connection_string(), vehicle_class=DroneTreeVehicle)

    for ii in range(20):
        sleep(1)
        print(ii)
        print(veh.distance_sensors[1])
        print(veh.battery)
        if ii == 4:
            veh.parameters['SIM_BATT_VOLTAGE'] = 12.11
        rng_cm = (ii+10)*10.
        print(rng_cm)
        veh.parameters['SIM_SONAR_SCALE'] = 0.0238/rng_cm
    veh.close()
    sitl.stop()


def test_nominal(copter4=True):
    """Test nominal case: should complete whole mission"""
    ca = ControlAutomaton(behaviour_tree)

    if copter4:
        # start own sitl and get CA to connect to it
        # will be copter 4
        sitl = start_sitl()
        ca.startup(override_args=[sitl.connection_string()])
    else:
        # run CA with 'sitl' argument - it'll run its own through dronekit_sitl
        # will be copter 3.3
        sitl = None
        ca.startup(override_args=['sitl'])

    for ii in range(100):
        ca.tick()
        sleep(1)
        if ca.vehicle.is_armable:
            break
    # should still be on the ground
    assert ca.vehicle.location.global_relative_frame.alt < 0.5
    assert ca.vehicle.is_armable
    ca.vehicle.arm()
    for ii in range(3):
        ca.tick()
        sleep(1)
    # should be on the ground
    assert ca.vehicle.location.global_relative_frame.alt < 0.2
    # switch to auto and lift the throttle to trigger auto takeoff
    ca.vehicle.mode = VehicleMode('AUTO')
    ca.vehicle.channels.overrides['3'] = 1700
    # log as we visit each waypoint
    wp_log = {2: False,
              4: False,
              7: False,
              10: False,
              13: False,
              16: False,
              18: False,
              20: False}
    for ii in range(200):
        ca.tick()
        if ca.vehicle.commands.next in wp_log.keys():
            wp_log[ca.vehicle.commands.next] = True
        print(wp_log)
        if ca.finished():
            break
        sleep(1)
    # should be back on the ground at HOME
    assert ca.vehicle.location.global_relative_frame.alt < 0.3
    # should have visited everywhere
    for wpn in wp_log.keys():
        assert wp_log[wpn], 'Missed WP {}'.format(wpn)
    # close down ControlAutomaton
    ca.cleanup()
    if sitl:
        sitl.stop()
    print("Test passed")


if __name__ == '__main__':
    test_nominal()
