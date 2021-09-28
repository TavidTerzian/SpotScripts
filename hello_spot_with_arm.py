# Copyright (c) 2021 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Tutorial to show how to use the Boston Dynamics API"""
from __future__ import print_function
import argparse
import sys
import time
import os
import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry

from bosdyn.client.image import ImageClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand


def hello_spot(config):
    """A simple example of using the Boston Dynamics API to command a Spot robot."""

    # The Boston Dynamics Python library uses Python's logging module to
    # generate output. Applications using the library can specify how
    # the logging information should be output.
    bosdyn.client.util.setup_logging(config.verbose)

    # The SDK object is the primary entry point to the Boston Dynamics API.
    # create_standard_sdk will initialize an SDK object with typical default
    # parameters. The argument passed in is a string identifying the client.
    sdk = bosdyn.client.create_standard_sdk('HelloSpotClient')

    # A Robot object represents a single robot. Clients using the Boston
    # Dynamics API can manage multiple robots, but this tutorial limits
    # access to just one. The network address of the robot needs to be
    # specified to reach it. This can be done with a DNS name
    # (e.g. spot.intranet.example.com) or an IP literal (e.g. 10.0.63.1)
    robot = sdk.create_robot(config.hostname)

    # Clients need to authenticate to a robot before being able to use it.
    robot.authenticate(config.username, config.password)

    # Establish time sync with the robot. This kicks off a background thread to establish time sync.
    # Time sync is required to issue commands to the robot. After starting time sync thread, block
    # until sync is established.
    robot.time_sync.wait_for_sync()

    #Check if robot has arm
    assert robot.has_arm(), "Robot requires an arm to run this example."

    # Verify the robot is not estopped and that an external application has registered and holds
    # an estop endpoint.
    assert not robot.is_estopped(), "Robot is estopped. Please use an external E-Stop client, " \
                                    "such as the estop SDK example, to configure E-Stop."

    # Only one client at a time can operate a robot. Clients acquire a lease to
    # indicate that they want to control a robot. Acquiring may fail if another
    # client is currently controlling the robot. When the client is done
    # controlling the robot, it should return the lease so other clients can
    # control it. Note that the lease is returned as the "finally" condition in this
    # try-catch-finally block.
    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    lease = lease_client.acquire()
    try:
        with bosdyn.client.lease.LeaseKeepAlive(lease_client):
            # Now, we are ready to power on the robot. This call will block until the power
            # is on. Commands would fail if this did not happen. We can also check that the robot is
            # powered at any point.
            robot.logger.info("Powering on robot... This may take several seconds.")
            robot.power_on(timeout_sec=20)
            assert robot.is_powered_on(), "Robot power on failed."
            robot.logger.info("Robot powered on.")

            # Tell the robot to stand up. The command service is used to issue commands to a robot.
            # The set of valid commands for a robot depends on hardware configuration. See
            # SpotCommandHelper for more detailed examples on command building. The robot
            # command service requires timesync between the robot and the client.
            robot.logger.info("Commanding robot to stand...")
            command_client = robot.ensure_client(RobotCommandClient.default_service_name)
            blocking_stand(command_client, timeout_sec=10)
            robot.logger.info("Robot standing.")
            time.sleep(3)

            # Do a joint move to move the arm around.
            sh0 = wrappers_pb2.DoubleValue(value=0.0692)
            sh1 = wrappers_pb2.DoubleValue(value=-1.882)
            el0 = wrappers_pb2.DoubleValue(value=1.652)
            el1 = wrappers_pb2.DoubleValue(value=-0.0691)
            wr0 = wrappers_pb2.DoubleValue(value=1.622)
            wr1 = wrappers_pb2.DoubleValue(value=1.550)

            # Build up a proto.
            joint_position1 = arm_command_pb2.ArmJointPosition(sh0=sh0, sh1=sh1, el0=el0, el1=el1,
                                                               wr0=wr0, wr1=wr1)

            traj_point = arm_command_pb2.ArmJointTrajectoryPoint(position=joint_position1)
            arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[traj_point])
            joint_move_command = arm_command_pb2.ArmJointMoveCommand.Request(
                trajectory=arm_joint_traj)
            arm_command = arm_command_pb2.ArmCommand.Request(
                arm_joint_move_command=joint_move_command)
            sync_arm = synchronized_command_pb2.SynchronizedCommand.Request(arm_command=arm_command)
            arm_sync_robot_cmd = robot_command_pb2.RobotCommand(synchronized_command=sync_arm)

            # Make the open gripper RobotCommand
            gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)

            # Combine the arm and gripper commands into one RobotCommand
            command = RobotCommandBuilder.build_synchro_command(gripper_command, arm_sync_robot_cmd)

            # Send the request
            command_client.robot_command(command)
            robot.logger.info('Moving arm to position 1.')

            time.sleep(4.0)

            # Log a comment.
            # Comments logged via this API are written to the robots test log. This is the best way
            # to mark a log as "interesting". These comments will be available to Boston Dynamics
            # devs when diagnosing customer issues.
            log_comment = "We have run the custom hello spot."
            robot.operator_comment(log_comment)
            robot.logger.info('Added comment "%s" to robot log.', log_comment)

            # Power the robot off. By specifying "cut_immediately=False", a safe power off command
            # is issued to the robot. This will attempt to sit the robot before powering off.
            robot.power_off(cut_immediately=False, timeout_sec=20)
            assert not robot.is_powered_on(), "Robot power off failed."
            robot.logger.info("Robot safely powered off.")
    finally:
        # If we successfully acquired a lease, return it.
        lease_client.return_lease(lease)


def _maybe_display_image(image, display_time=3.0):
    """Try to display image, if client has correct deps."""
    try:
        from PIL import Image
        import io
    except ImportError:
        logger = bosdyn.client.util.get_logger()
        logger.warning("Missing dependencies. Can't display image.")
        return
    try:
        image = Image.open(io.BytesIO(image.data))
        image.show()
        time.sleep(display_time)
    except Exception as exc:
        logger = bosdyn.client.util.get_logger()
        logger.warning("Exception thrown displaying image. %r", exc)

def _maybe_save_image(image, path):
    """Try to save image, if client has correct deps."""
    logger = bosdyn.client.util.get_logger()
    try:
        from PIL import Image
        import io
    except ImportError:
        logger.warning("Missing dependencies. Can't save image.")
        return
    name = "hello-spot-img.jpg"
    if path is not None and os.path.exists(path):
        path = os.path.join(os.getcwd(), path)
        name = os.path.join(path, name)
        logger.info("Saving image to: {}".format(name))
    else:
        logger.info("Saving image to working directory as {}".format(name))
    try:
        image = Image.open(io.BytesIO(image.data))
        image.save(name)
    except Exception as exc:
        logger = bosdyn.client.util.get_logger()
        logger.warning("Exception thrown saving image. %r", exc)


def main(argv):
    """Command line interface."""
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_common_arguments(parser)
    parser.add_argument('-s', '--save', action='store_true', help='Save the image captured by Spot to the working directory. To chose the save location, use --save_path instead.')
    parser.add_argument('--save-path', default=None, nargs='?', help='Save the image captured by Spot to the provided directory. Invalid path saves to working directory.')
    options = parser.parse_args(argv)
    try:
        hello_spot(options)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.error("Hello, Spot! threw an exception: %r", exc)
        return False


if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
