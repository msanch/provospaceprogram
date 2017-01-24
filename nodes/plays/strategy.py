import util
import tactics

strategies = enum(FOLLOW_BALL)


def pick_play():
	return strategies.FOLLOW_BALL


def make_play():
	play = pick_play()
	if play == strategies.FOLLOW_BALL:
		tactics.follow_ball()
	else:
		pass # ros log error