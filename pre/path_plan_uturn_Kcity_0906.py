#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import rospy

from tracking_msg.msg import TrackingObjectArray
from erp_driver.msg import erpCmdMsg, erpStatusMsg
from std_msgs.msg import Float32, String, Float32MultiArray, Int32


class Cone:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class ConeVizNode:
    def __init__(self):
        rospy.init_node('cone_target_ctrl')

        # params
        self.right_offset_m = rospy.get_param('~right_offset_m', 2.0)   # 오른쪽 오프셋 [m]
        self.max_forward_x  = rospy.get_param('~max_forward_x', 5.0)
        self.wheelbase      = rospy.get_param('~wheelbase', 1.0)        # [m]
        self.follow_dist_m  = rospy.get_param('~follow_dist_m', 0.5)    # 경로 시작으로부터 추종 거리 [m]

        # 상태 (최소화)
        self.target_x = None
        self.target_y = None
        self.is_status = True       # 필요 시 초기값 조정
        self.erpStatus_msg = None
        self.is_desiredVel = False
        # ERP42 제어 publisher
        self.uturn_steer_pub = rospy.Publisher("/uturn_steer", Int32, queue_size=1)
        self.erp_msg = erpCmdMsg()
        self.erp_msg.gear = 2        # 차량 세팅에 맞춰 조정
        self.erp_msg.speed = 10
        self.erp_msg.brake = 0

        # subscribers
        rospy.Subscriber('/fusion_box/fused_3d_box',
                         TrackingObjectArray,
                         self.lidar_callback,
                         queue_size=1)
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)


    # ---------------------- core logic ----------------------
    def lidar_callback(self, msg: TrackingObjectArray):

        if self.Path_state == "Rubber_cone_path":
        # 1) 라바콘 추출 + 필터
            cones = []
            for obj in msg.array:
                if not hasattr(obj, "bbox") or not hasattr(obj.bbox, "points") or len(obj.bbox.points) < 8:
                    continue

                color = getattr(obj, "type_id", None)   # 예: 12/13이 콘
                if color not in (12, 13):
                    continue

                if not hasattr(obj, "point"):
                    continue
                cx, cy = obj.point.x, obj.point.y

                # 전방/측면 범위(필요 시 활성화)
                # if not (0.0 < cx < self.max_forward_x and abs(cy) <= 3.0):
                #     continue

                cones.append(Cone(cx, cy))

            # 2) 거리 기준 정렬
            cones.sort(key=lambda c: math.hypot(c.x, c.y))

            # 3) 타겟/경로 계산
            if not cones:
                self.target_x, self.target_y = None, None
                steer, delta, kappa = 0, 0.0, 0.0
            elif len(cones) == 1:
                c = cones[0]
                theta = math.atan2(c.y, c.x)
                perp_right = theta - math.pi / 2.0
                self.target_x = c.x + self.right_offset_m * math.cos(perp_right)
                self.target_y = c.y + self.right_offset_m * math.sin(perp_right)
                steer, delta, kappa = self.cal_pp_steer(self.target_x, self.target_y)
            else:
                k = min(5, len(cones))
                sel = cones[:k]

                path_pts = []
                for i in range(len(sel) - 1):
                    c1, c2 = sel[i], sel[i + 1]
                    mid_x = (c1.x + c2.x) / 2.0
                    mid_y = (c1.y + c2.y) / 2.0
                    seg_angle = math.atan2(c2.y - c1.y, c2.x - c1.x)
                    perp_right = seg_angle - math.pi / 2.0
                    px = mid_x + self.right_offset_m * math.cos(perp_right)
                    py = mid_y + self.right_offset_m * math.sin(perp_right)
                    path_pts.append((px, py))

                tgt = self._point_along_path(path_pts, self.follow_dist_m) if path_pts else None
                if tgt is None:
                    self.target_x, self.target_y = None, None
                    steer, delta, kappa = 0, 0.0, 0.0
                else:
                    self.target_x, self.target_y = tgt
                    steer, delta, kappa = self.cal_pp_steer(self.target_x, self.target_y)

            self.uturn_steer_pub.publish(steer)

        else:
            return
    # ---------------------- Pure Pursuit ----------------------
    def cal_pp_steer(self, x, y):
        if x is None or y is None:
            return 0, 0.0, 0.0
        L = self.wheelbase
        eps = 1e-6
        kappa = 2.0 * y / (x * x + y * y + eps)   # 순수추종 등가 곡률
        delta = math.atan(L * kappa)
        servo = -int(2000 * (delta / 0.4922))     # 차량 캘리브레이션에 맞춰 조정
        servo = max(min(servo, 2000), -2000)
        return servo, delta, kappa


    def status_callback(self, msg):
        self.is_status = True
        self.erpStatus_msg = msg

    # ---------------------- helpers ----------------------
    def _point_along_path(self, points, distance):
        if not points:
            return None
        if len(points) == 1:
            return points[0]

        d_acc = 0.0
        for i in range(len(points) - 1):
            x1, y1 = points[i]
            x2, y2 = points[i + 1]
            seg = math.hypot(x2 - x1, y2 - y1)
            if seg <= 1e-6:
                continue
            if d_acc + seg >= distance:
                t = (distance - d_acc) / seg
                return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
            d_acc += seg
        return points[-1]

    # ---------------------- spin ----------------------
    def spin(self):
        rospy.loginfo("cone_target_ctrl running. Publishing steer to /erp42_ctrl_cmd")
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ConeVizNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
