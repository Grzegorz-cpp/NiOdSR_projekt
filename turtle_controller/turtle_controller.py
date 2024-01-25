#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import cv2
import numpy as np


class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.get_logger().info(
            "\nKontroler Turtlesim rozpoczal dzialanie!\nWcisnij ESC aby zakonczyc program\nNacisinij myszka na plansze aby sterowac Turtlesim")

        # -----------------------Stale-----------------------

        # Predkosc poruszania sie
        self.DEFAULT_LINEAR_VELOCITY = 1.0
        self.ANGULAR_VELOCITY = 2.0

        # Dopuszczalna odleglosc od celu
        self.DISTANCE = 0.1

        # Odleglosc ucieczki
        self.DISTANCE_MIN = 1.5
        self.SAFE_DISTANCE = 1.5

        # Wymiary okna i siatki
        self.WINDOW_SIZE = 500
        self.GRID_SIZE = 11

        # Kolor tla, siatki i strzalki
        self.BACKGROUND_COLOR = (0, 0, 0)
        self.GRID_COLOR = (192, 192, 192)
        self.ARROW_COLOR = (0, 255, 0)

        # Wymiary okna turtlesim
        self.TURTLESIM_WINDOW_WIDTH = 11.0
        self.TURTLESIM_WINDOW_HEIGHT = 11.0

        # -----------------------Zmienne-----------------------

        # Pozycja startowa
        self.mouse_x = 5.5
        self.mouse_y = 5.5

        self.window_closed = False
        self.escape = False
        self.lbutton = False
        self.target_reached = False

        cv2.namedWindow("Turtlesim Control")
        cv2.setMouseCallback("Turtlesim Control", self.mouse_callback)

    def pose_callback(self, pose: Pose):
        cmd = Twist()

        # -----------------------Logika sterowania turtlesim-----------------------

        # Odleglosc od celu
        target_distance = np.sqrt((self.mouse_x - pose.x)**2 + (self.mouse_y - pose.y)**2)

        # Kierunek do celu
        target_angle = np.arctan2(self.mouse_y - pose.y, self.mouse_x - pose.x)
        angle_diff = target_angle - pose.theta

        # Uwzglednij katy wieksze niz 180 stopni
        if abs(angle_diff) > np.pi:
            if angle_diff > 0:
                angle_diff -= 2 * np.pi
            else:
                angle_diff += 2 * np.pi

        # Sprawdz czy cel nie za blisko
        if self.lbutton and target_distance < self.DISTANCE_MIN and abs(angle_diff) > np.pi / 6:
            self.escape = True
            self.lbutton = False
        elif self.lbutton and target_distance < self.DISTANCE_MIN / 3 and abs(angle_diff) > np.pi / 12:
            self.escape = True
            self.lbutton = False
        else:
            self.lbutton = False

        # Odsun sie jesli cel za blisko
        if self.escape:
            cmd.linear.x = self.DEFAULT_LINEAR_VELOCITY

        # Koniec ucieczki
        if self.escape and target_distance > self.SAFE_DISTANCE:
            self.escape = False

        # Warunek zatrzymania
        if target_distance < self.DISTANCE and not self.escape:
            cmd.linear.x = 0.0
            if self.target_reached:
                self.get_logger().info("Cel osiagniety")
                self.target_reached = False
        elif not self.escape:
            # Predkosc liniowa
            cmd.linear.x = self.DEFAULT_LINEAR_VELOCITY
            # Predkosc katowa
            cmd.angular.z = self.ANGULAR_VELOCITY * angle_diff

            self.target_reached = True

        # Publikowanie predkosci liniowej i katowej turtlesim
        self.cmd_vel_pub_.publish(cmd)

        # -----------------------Grafika-----------------------

        # Rysowanie tla
        img = np.zeros((self.WINDOW_SIZE, self.WINDOW_SIZE, 3), np.uint8)
        cv2.rectangle(img, (0, 0), (self.WINDOW_SIZE, self.WINDOW_SIZE), self.BACKGROUND_COLOR, thickness=cv2.FILLED)

        # Rysowanie siatki
        for i in range(1, self.GRID_SIZE):
            line_x = int((i / self.GRID_SIZE) * self.WINDOW_SIZE)
            line_y = int((i / self.GRID_SIZE) * self.WINDOW_SIZE)

            cv2.line(img, (line_x, 0), (line_x, self.WINDOW_SIZE), self.GRID_COLOR, 1)
            cv2.line(img, (0, line_y), (self.WINDOW_SIZE, line_y), self.GRID_COLOR, 1)

        # Rysowanie strzalki
        turtlesim_x = int((pose.x / self.TURTLESIM_WINDOW_WIDTH) * img.shape[1])
        turtlesim_y = int(img.shape[0] - (pose.y / self.TURTLESIM_WINDOW_HEIGHT) * img.shape[0])
        turtlesim_mouse_x = int((self.mouse_x / self.TURTLESIM_WINDOW_WIDTH) * img.shape[1])
        turtlesim_mouse_y = int(img.shape[0] - (self.mouse_y / self.TURTLESIM_WINDOW_HEIGHT) * img.shape[0])
        cv2.arrowedLine(img, (turtlesim_x, turtlesim_y), (turtlesim_mouse_x, turtlesim_mouse_y),self.ARROW_COLOR , 2)

        # Zamykanie okna
        cv2.imshow("Turtlesim Control", img)
        key = cv2.waitKey(1)
        if key == 27:
            self.window_closed = True

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Przeliczanie wspolrzednych klikniecia na uklad wspolrzednych turtlesim
            self.mouse_x = (x / self.WINDOW_SIZE) * self.TURTLESIM_WINDOW_WIDTH
            self.mouse_y = ((self.WINDOW_SIZE - y) / self.WINDOW_SIZE) * self.TURTLESIM_WINDOW_HEIGHT

            # Napisz wspolrzedne docelowe
            self.get_logger().info("Cel x: " + str(np.round(self.mouse_x, decimals = 2)) + ", y: " + str(np.round(self.mouse_y, decimals = 2)))

            self.lbutton = True

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    while not node.window_closed:
        rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

