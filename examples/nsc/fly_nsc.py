# -*- coding: utf-8 -*-
"""
This example uses an inspection scenario to illustrate the use of the
flight_idioms library in drone_trees to execute a mission.

Note that this does not require any direct import of py_trees libraries: all
the plumbing is provided by flight_idioms, stitching together behaviours from
the leaf_nodes library.

Usage:
    python fly_nsc.py render
        draws the behaviour tree in DOT, PNG and SVG
    python fly_nsc.py sitl
        launches and connects to a SITL simulator
    python fly_nsc.py <connection>
        Connects to external MAVLINK interface, e.g. drone telemetry or
        back door of GCS.

        For example, run Mission Planner, launch a copter simulator from the
        Simulation menu, open TCP host 14550 from the Ctrl+F Temp menu, and
        then run python fly_nsc.py tcp:127.0.0.1:14550.

Mission will start with preflight checks and then wait for pilot to arm, set
mode to Auto, and take-off (just lift the throttle as first waypoint is TO).
Mission then proceeds through waypoints, provided specific distance clearance
preconditions are satisfied at waypoints 4, 7 and 10.  Mission will skip
to returning home if they''re not, or if the GPS or battery levels fail
during the mission.

See also test_nsc.py, the test script for this controller,
and executable_mission.txt, the required waypoint file.

"""

from drone_trees import leaf_nodes as lf
from drone_trees import flight_idioms as im
from drone_trees.mission_handler import MissionHandler
from drone_trees.control_automaton import ControlAutomaton


def behaviour_tree(vehicle):
    """
    Construct the behaviour tree for the New Safe Confinement example.

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        Provides MAVLINK connection to drone

    Returns
    -------
    bt: py_trees.behaviour.Behaviour
        The node at the root of behaviour tree

    """

    # load the associated mission file
    mission_handler = MissionHandler('executable_mission.txt')

    # preflight: not in AUTO mode, failsafe configuration, upload mission,
    # verify mission upload, availability of distance sensor, basic GPS,
    # EKF healthy, armable
    preflight_behaviours = [lf.CheckModeNot(vehicle, 'AUTO'),
                            lf.CheckParam(vehicle, 'FS_THR_ENABLE', 2),
                            lf.CheckParam(vehicle, 'FS_GCS_ENABLE', 2),
                            mission_handler.upload_mission(vehicle),
                            lf.MissionVerify(vehicle),
                            lf.IsDistSensorAvbl(vehicle, 1),
                            lf.CheckGPS(vehicle, 2),
                            lf.CheckEKF(vehicle),
                            lf.IsArmable(vehicle)]

    # safety check: return home via SAFTI point any time if battery <30%
    safety_low_battery = im.safety_module(name="Low battery",
                                          check=lf.BatteryLevelAbove(vehicle, 30),
                                          fallback=mission_handler.go_safti(vehicle),
                                          oneshot=True)

    # safety check: minimum distance criterion of 2m
    safety_avoidance = im.safety_module(name="Collision avoidance",
                                        check=lf.CheckDistance(vehicle, 1, 2.),
                                        fallback=mission_handler.go_safti(vehicle))

    # safety check: come home via SAFTI any time if EKF bad
    safety_ekf = im.safety_module(name="EKF health",
                                  check=lf.CheckEKF(vehicle),
                                  fallback=mission_handler.go_safti(vehicle))

    # leg handlers: for each of the jumps in the waypoint file (6,9,12,15)
    # define preconditions for proceeding
    # based only on distance clearance in this example
    # TODO get SITL to mimic rangefinder and put clearance checks in
    # proceed from WP 4 to 7 if 45m < distance ahead < 55m
    leg_4_7 = im.leg_handler(vehicle, 4, 7,
                             preconds=[lf.CheckDistBounded(vehicle, 1, 45., 55.)])
    # proceed from WP 7 to 10 if 15m < distance ahead < 25m
    leg_7_10 = im.leg_handler(vehicle, 7, 10,
                              preconds=[lf.CheckDistBounded(vehicle, 1, 15., 25.)])
    # proceed from WP 10 to 13 if 8m < distance ahead < 12m
    leg_10_13 = im.leg_handler(vehicle, 10, 13,
                               preconds=[lf.CheckDistBounded(vehicle, 1, 8., 12.)])
    # proceed from WP 13 to 16 if 4m < distance ahead < 6m
    leg_13_16 = im.leg_handler(vehicle, 13, 16,
                               preconds=[lf.CheckDistBounded(vehicle, 1, 4., 6.)])
    # proceed from WP 16 to 18 with no preconditions
    leg_16_18 = im.leg_handler(vehicle, 16, 18)

    # combine all modules into a single flight manager behavour tree
    root_node = im.flight_manager(vehicle,
                                  preflight=preflight_behaviours,
                                  safety=[safety_ekf,
                                          safety_low_battery,
                                          safety_avoidance],
                                  legs=[leg_4_7,
                                        leg_7_10,
                                        leg_10_13,
                                        leg_13_16,
                                        leg_16_18])

    return root_node


if __name__ == "__main__":
    # run the behaviour tree
    # drone starts at Clifton Bridge in SITL mode
    APP = ControlAutomaton(behaviour_tree,
                           sitl_lat=51.454531, sitl_lon=-2.629158)
    APP.main()
