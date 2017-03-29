#!/usr/bin/env python
import curses
import math

import rospy
from geometry_msgs.msg import Pose2D


class KeyboardController(object):
    def __init__(self):
        self._setup_curses()
        self._setup_ros()
        self.deltas = (0, 0, 0)
        self.velocity = 0

    def _setup_curses(self):
        self.stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        self.stdscr.keypad(1)

    def _take_down_curses(self):
        curses.nocbreak()
        self.stdscr.keypad(0)
        curses.echo()
        curses.endwin()

    def _setup_ros(self):
        rospy.init_node("psp_keyboard_controller", anonymous=False)
        rospy.Subscriber("/provospaceprogram_home/ally1_estimator", Pose2D,
                self.handle_current_state)
        self.publisher = rospy.Publisher("psp_keyboard_desired", Pose2D, queue_size=10)

    def handle_current_state(self, msg):
        self.publisher.publish(
                    x=(self.deltas[0] + msg.x),
                    y=(self.deltas[1] + msg.y),
                    theta=self.deltas[2]
                )

    def run(self):
        self.stdscr.addstr(0,0,"Hit 'q' to quit")
        self.stdscr.refresh()
        key = ''
        while key != ord('q'):
            key = self.stdscr.getch()
            if key == curses.KEY_UP:
                self.deltas = (0, self.velocity, math.pi/2)
            elif key == curses.KEY_DOWN:
                self.deltas = (0, -self.velocity, -math.pi/2)
            elif key == curses.KEY_LEFT:
                self.deltas = (-self.velocity, 0, math.pi)
            elif key == curses.KEY_RIGHT:
                self.deltas = (self.velocity, 0, 0)
            elif ord('0') <= key <= ord('9'):
                self.velocity = key - ord('0')
            self.stdscr.erase()
            self.stdscr.addstr(0,0,"Hit 'q' to quit")
            self.stdscr.addstr(2, 0, str(self.deltas))
            self.stdscr.refresh()
        self._take_down_curses


def main():
    controller = KeyboardController()
    controller.run()


if __name__ == '__main__':
    main()

