#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D

from soccerobjects import *
from soccerref.msg import GameState as GameStateMsg
import strategy


def main():
    rospy.init_node('ai', anonymous=False)

    # Get params
    is_player1 = rospy.get_param('~is_player1')
    is_team_home = rospy.get_param('~is_team_home')

    # Create all soccer objects
    me   = Robot(is_player1=is_player1)
    ally = Robot(is_player1=(not is_player1))
    opp1 = Robot()
    opp2 = Robot()
    ball = Point2D()
    game_state = GameState(reset_field=False, play=True)

    # Subscribe to Robot and Ball positions
    rospy.Subscriber('me',   Pose2D, me.update)
    rospy.Subscriber('ally', Pose2D, ally.update)
    rospy.Subscriber('ball_estimator', Pose2D, ball.update)
    rospy.Subscriber('opp1', Pose2D, opp1.update)
    rospy.Subscriber('opp2', Pose2D, opp2.update)
    rospy.Subscriber('game', GameStateMsg, game_state.update)

    coach = strategy.Strategy(game_state, is_player1)

    rate = rospy.Rate(100) # 100 Hz
    while not rospy.is_shutdown():
        try:
            if game_state.reset_field:
                is_home_now = is_team_home ^ game_state.second_half
                if (is_home_now and game_state.home_penalty) or (not is_home_now and game_state.away_penalty):
                    coach.penalty(me)
                else:
                    coach.return_to_start(me)
            elif game_state.play:
                coach.make_play(me, ally, ball)
        except SoccerResetException:
            pass
        rate.sleep()


if __name__ == '__main__':
    main()
