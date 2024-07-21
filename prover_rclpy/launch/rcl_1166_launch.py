import os
import signal

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
import launch.events
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_service import LaunchService
import launch_ros
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from multiprocessing import Process
from typing import List


def up_launch(launch_description_list: List[LaunchDescriptionEntity]):
    ls = LaunchService()
    ls.include_launch_description(LaunchDescription(launch_description_list))
    ls.run()


def generate_launch_description(ns):
    lc_node = LifecycleNode(
        package='lifecycle_py',
        executable='lifecycle_talker',
        name='lc_talker',
        namespace=ns,
        output='screen',
    )

    # send activate when on_configure finished
    emit_active = RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lc_node,
            goal_state='inactive',
            # start_state='configuring',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lc_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ))

    # Prepare the talker node, and request transition to be configured.
    emit_configure = RegisterEventHandler(
        OnProcessStart(
            target_action=lc_node,
            on_start=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lc_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )),
            ]))


    return [lc_node, emit_active, emit_configure]


if __name__ == '__main__':
    os.setpgrp()
    try:
        for i in range(12):
            ls = generate_launch_description('room'+str(i))
            launch_proc = Process(target=up_launch,
                                  args=(ls,),
                                  )
            launch_proc.start()
    except KeyboardInterrupt:
        print('group kill')
        os.killpg(0, signal.SIGINT)