#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#==============9/19 유턴 시각화 추가 버전 ==========

import math
import rospy

from tracking_msg.msg import TrackingObjectArray
from erp_driver.msg import erpCmdMsg, erpStatusMsg

from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float32, String, Float32MultiArray, Int32


class Cone:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class ConeVizNode:
    def __init__(self):
        rospy.init_node('cone_target_ctrl')

        # params
        self.right_offset_m = rospy.get_param('~right_offset_m', 1.5)   # 오른쪽 오프셋 [m]
        self.max_forward_x  = rospy.get_param('~max_forward_x', 5.0)
        self.wheelbase      = rospy.get_param('~wheelbase', 1.0)        # [m]
        self.follow_dist_m  = rospy.get_param('~follow_dist_m', 0.5)    # 경로 시작으로부터 추종 거리 [m]

        # 상태 (최소화)
        self.target_x = None
        self.target_y = None
        self.is_status = True
        self.erpStatus_msg = None

        # ERP42 제어 publisher
        self.uturn_steer_pub = rospy.Publisher("/uturn_steer", Int32, queue_size=1)
        self.erp_msg = erpCmdMsg()
        self.erp_msg.gear = 2        # 차량 세팅에 맞춰 조정
        self.erp_msg.speed = 10
        self.erp_msg.brake = 0

        # RViz 시각화 publisher
        self.viz_pub  = rospy.Publisher("/cone_target_viz", MarkerArray, queue_size=1)
        self.path_pub = rospy.Publisher("/cone_target_path", Path, queue_size=1)

        # subscribers
        rospy.Subscriber('/fusion_box/fused_3d_box',
                         TrackingObjectArray,
                         self.lidar_callback,
                         queue_size=1)
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)

    # ---------------------- core logic ----------------------
    def lidar_callback(self, msg: TrackingObjectArray):
        # frame 결정
        frame_id = getattr(getattr(msg, "header", None), "frame_id", "") or "base_link"

        # 1) 라바콘 추출 + 필터
        cones = []
        for obj in msg.array:
            # 기본 유효성
            if not hasattr(obj, "bbox") or not hasattr(obj.bbox, "points") or len(obj.bbox.points) < 8:
                continue
            # 색상(타입) 필터: 예 12/13이 콘
            color = getattr(obj, "type_id", None)
            if color not in (12, 13):
                continue
            if not hasattr(obj, "point"):
                continue

            cx, cy = obj.point.x, obj.point.y

            # 전방/측면 범위(필요 시 활성화)
            # if not (0.0 < cx < self.max_forward_x and abs(cy) <= 3.0):
            #     continue

            # 현재는 y>0 (차량 좌측) 콘만 사용
            if cy > 0:
                cones.append(Cone(cx, cy))
            else:
                continue

        # [NEW] 1-1) 매우 가까운 콘(<= 0.01 m) 중복 제거
        cones = self._dedup_cones(cones, tol=0.01)

        # 2) 거리 기준 정렬
        cones.sort(key=lambda c: math.hypot(c.x, c.y))

        # 3) 타겟/경로 계산
        path_pts = []
        if not cones:
            self.target_x, self.target_y = None, None
            steer, delta, kappa = 0, 0.0, 0.0

        elif len(cones) == 1:
            # 차량 기준 오른쪽(-Y)으로 오프셋
            c = cones[0]
            # base_link 기준: x 전방(+), y 좌측(+), 따라서 오른쪽은 -Y
            self.target_x = float(c.x)
            self.target_y = float(c.y - self.right_offset_m)
            # 시각화를 위해 단일 타겟만 있는 경우 간단히 포인트 1개를 경로로
            path_pts = [(self.target_x, self.target_y)]
            steer, delta, kappa = self.cal_pp_steer(self.target_x, self.target_y)

        else:
            k = min(5, len(cones))
            print(len(cones))
            sel = cones[:k]

            for i in range(len(sel) - 1):
                c1, c2 = sel[i], sel[i + 1]
                mid_x = (c1.x + c2.x) / 2.0
                mid_y = (c1.y + c2.y) / 2.0
                seg_angle = math.atan2(c2.y - c1.y, c2.x - c1.x)
                # 세그먼트 기준 오른쪽(트랙 내부)으로 오프셋
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

        # 4) ERP42 제어 퍼블리시
        
        self.uturn_steer_pub.publish(steer)


        # 5) RViz 시각화 퍼블리시
        self.publish_viz(path_pts, (self.target_x, self.target_y), frame_id)

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
            if seg <= 0.1:
                continue
            if d_acc + seg >= distance:
                t = (distance - d_acc) / seg
                return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
            d_acc += seg
        return points[-1]

    # [NEW] 가까운 콘 중복 제거 (첫 번째로 본 콘 유지)
    def _dedup_cones(self, cones, tol=0.01):
        """
        cones: [Cone]
        tol: 같은 콘으로 간주할 거리 임계값 [m]. 기본 0.01 m.
        """
        if not cones:
            return []
        kept = []
        tol2 = tol * tol
        for c in cones:
            dup = False
            for k in kept:
                dx = c.x - k.x
                dy = c.y - k.y
                if (dx*dx + dy*dy) <= tol2:
                    dup = True
                    break
            if not dup:
                kept.append(c)
        return kept

    # ---------------------- RViz publishers ----------------------
    def publish_viz(self, path_pts, target_xy, frame_id="base_link"):
        """
        path_pts: [(x,y), ...]
        target_xy: (x,y) or (None, None)
        """
        # 1) Path 메시지
        path_msg = Path()
        path_msg.header.frame_id = frame_id
        path_msg.header.stamp = rospy.Time.now()
        for (x, y) in path_pts:
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.header.stamp = path_msg.header.stamp
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

        # 2) MarkerArray (LINE_STRIP + SPHERE + TEXT_VIEW_FACING)
        arr = MarkerArray()

        # (a) 경로 라인
        if path_pts:
            line = Marker()
            line.header.frame_id = frame_id
            line.header.stamp = rospy.Time.now()
            line.ns = "cone_target"
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.08  # 라인 두께 [m]
            line.color.r = 0.1
            line.color.g = 0.6
            line.color.b = 1.0
            line.color.a = 1.0
            line.pose.orientation.w = 1.0
            for (x, y) in path_pts:
                p = Point(x=x, y=y, z=0.0)
                line.points.append(p)
            arr.markers.append(line)

        # (b) 타겟 포인트 (구 + 텍스트)
        tx, ty = target_xy
        if tx is not None and ty is not None:
            sphere = Marker()
            sphere.header.frame_id = frame_id
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = "cone_target"
            sphere.id = 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.scale.x = 0.3
            sphere.scale.y = 0.3
            sphere.scale.z = 0.3
            sphere.color.r = 1.0
            sphere.color.g = 0.2
            sphere.color.b = 0.2
            sphere.color.a = 1.0
            sphere.pose.position.x = tx
            sphere.pose.position.y = ty
            sphere.pose.position.z = 0.0
            sphere.pose.orientation.w = 1.0
            arr.markers.append(sphere)

            text = Marker()
            text.header.frame_id = frame_id
            text.header.stamp = rospy.Time.now()
            text.ns = "cone_target"
            text.id = 2
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.scale.z = 0.3
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.pose.position.x = tx
            text.pose.position.y = ty
            text.pose.position.z = 0.4
            text.pose.orientation.w = 1.0
            text.text = "TARGET"
            arr.markers.append(text)

        # (c) 아무것도 없으면 지우기
        if not path_pts and (tx is None or ty is None):
            clear = Marker()
            clear.action = Marker.DELETEALL
            arr.markers.append(clear)

        self.viz_pub.publish(arr)

    # ---------------------- spin ----------------------
    def spin(self):
        rospy.loginfo("cone_target_ctrl running. Publishing steer, path, and viz")
        rospy.spin()


if __name__ == '__main__':
    try:
        node = ConeVizNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
