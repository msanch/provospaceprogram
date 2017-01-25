#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from soccerref.msg import GameState

import soccerobjects
import strategy


def update(obj, msg):
    obj.update(msg)


def main():
    rospy.init_node('ai', anonymous=False)

    # Get params
    is_team_home = rospy.get_param('~is_team_home', True)
    is_player1 = rospy.get_param('~is_player1', True)

    # Create all soccer objects
    me = soccerobjects.Robot(player1=is_player1)
    ally = soccerobjects.Robot(player1=is_player1)
    opp1 = soccerobjects.Robot()
    opp2 = soccerobjects.Robot()
    ball = soccerobjects.Ball()
    # Subscribe to Robot and Ball positions
    rospy.Subscriber('me',   Pose2D, lambda msg: update(me, msg))
    rospy.Subscriber('ally', Pose2D, lambda msg: update(ally, msg))
    rospy.Subscriber('ball', Pose2D, lambda msg: update(ball, msg))
    rospy.Subscriber('opp1', Pose2D, lambda msg: update(opp1, msg))
    rospy.Subscriber('opp2', Pose2D, lambda msg: update(opp2, msg))

    # This message comes from the soccerref and
    # tells us if we should be playing or not
    # rospy.Subscriber('/game_state', GameState, _handle_game_state)

    coach = strategy.Strategy()

    rate = rospy.Rate(100) # 100 Hz
    while not rospy.is_shutdown():
        coach.make_play(me, ally, ball)
        rate.sleep()


if __name__ == '__main__':
    # If this file was run from the command line, then do the following:
    main()
