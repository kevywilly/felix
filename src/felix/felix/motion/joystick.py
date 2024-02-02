import math
from typing import Optional
from enum import Enum
from geometry_msgs.msg import Twist

class JoystickEventType:
    move = "move"
    stop = "stop"
    start = "start"

class JoystickDirection:
    FORWARD = "FORWARD"
    BACKWARD = "BACKWARD"
    LEFT = "LEFT"
    RIGHT = "RIGHT"

class JoystickUpdateEvent:
    def __init__(
            self, 
            type: str, 
            x: float = 0.0, 
            y: float = 0.0, 
            distance: float = 0.0,
            direction: Optional[str] = None,
            ):
        
        self.type = type
        self.x = x
        self.y = y
        self.direction = direction
        self.distance: float = distance

    def get_twist(self) -> Twist:
        t = Twist()
        if self.type == JoystickEventType.move:
            try:
                dist = self.distance/100.0
                angle = math.degrees(math.atan2(self.x,self.y))
                x = self.x * dist
                y = self.y * dist
                if abs(angle) <= 10 or abs(angle) >=170:
                    # forward / backward
                    t.linear.x = y
                elif abs(angle) <= 85 or abs(angle) >=95:
                    # turn
                    t.linear.x = y
                    t.angular.z = -x
                else:
                    # slide
                    # t.linear.x = y
                    t.linear.y = -x
            except Exception as ex:
                t = Twist()
                print(ex.__str__())
        return t
    
    def dict(self):
        return self.__dict__