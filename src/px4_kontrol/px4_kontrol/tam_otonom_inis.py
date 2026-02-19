import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class TeknofestAvcisi(Node):
    def __init__(self):
        super().__init__('teknofest_avcisi_node')
        self.br = CvBridge()
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        # ---------------- TEKNOFEST GÖREV KURALLARI ----------------
        self.hedef_irtifa = 4.0         
        self.inis_irtifasi = 0.8        
        
        # --- QR OKUYUCU MOTORU ---
        self.qr_detector = cv2.QRCodeDetector()
        self.okunan_gorev = ""
        
        # ---------------- HİZALAMA ALGORİTMASI ----------------
        self.kp = 0.003  
        self.kd = 0.006  
        self.max_hiz = 0.8  
        
        # ---------------- DURUM MAKİNESİ VE HAFIZA ----------------
        self.durum = "KALKIS"  
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
        self.current_alt = 0.0  
        self.current_yaw = 0.0  
        self.target_found = False
        
        self.err_x = 0.0; self.err_y = 0.0
        self.prev_err_x = 0.0; self.prev_err_y = 0.0
        
        self.hedef_alani = 0.0
        self.dinamik_tolerans = 25  
        
        self.kayip_sayaci = 0
        self.son_vx_body = 0.0
        self.son_vy_body = 0.0
        self.offboard_counter = 0

        self.img_sub = self.create_subscription(Image, '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image', self.image_callback, qos_profile)
        self.pose_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.pose_callback, qos_profile)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_callback, qos_profile)

        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.timer = self.create_timer(0.05, self.control_loop)
        print('🏆 TEKNOFEST AJANI V11 AKTİF! Mod Savaşı Çözüldü, Kusursuz İniş Devrede.')

    def status_callback(self, msg): 
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def pose_callback(self, msg): 
        self.current_alt = -msg.z 
        self.current_yaw = msg.heading  

    def image_callback(self, msg):
        try:
            cv_img = self.br.imgmsg_to_cv2(msg, "bgr8")
            data, bbox, _ = self.qr_detector.detectAndDecode(cv_img)
            
            h, w, _ = cv_img.shape
            cx, cy = w // 2, h // 2
            self.target_found = False
            self.okunan_gorev = data if data else "GOREV_YOK"
            
            if bbox is not None and len(bbox) > 0:
                pts = np.int32(bbox[0])
                cv2.polylines(cv_img, [pts], True, (0, 0, 255), 2)
                
                if data == "TEKNOFEST_INIS":
                    x, y, bw, bh = cv2.boundingRect(pts)
                    self.hedef_alani = bw * bh
                    obj_cx = x + bw // 2; obj_cy = y + bh // 2
                    
                    self.err_x = float(obj_cx - cx); self.err_y = float(obj_cy - cy)
                    self.target_found = True
                    self.dinamik_tolerans = max(20, int(np.sqrt(self.hedef_alani) * 0.2))
                    
                    cv2.polylines(cv_img, [pts], True, (0, 255, 0), 4)
                    cv2.circle(cv_img, (cx, cy), self.dinamik_tolerans, (255, 255, 255), 1)
                    cv2.line(cv_img, (cx, cy), (obj_cx, obj_cy), (0, 255, 255), 2) 
                    cv2.putText(cv_img, f"[GOREV: INIS]", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.putText(cv_img, f"MOD: {self.durum}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(cv_img, f"ALT: {self.current_alt:.1f}m | YAW: {math.degrees(self.current_yaw):.0f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(cv_img, f"OKUNAN: {self.okunan_gorev}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            cv2.imshow("Teknofest Gorev Ekrani", cv_img)
            cv2.waitKey(1)
        except: pass

    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand()
        msg.command = command; msg.param1 = kwargs.get("param1", 0.0); msg.param2 = kwargs.get("param2", 0.0)
        msg.target_system = 1; msg.target_component = 1; msg.source_system = 1; msg.source_component = 1; msg.from_external = True
        self.command_pub.publish(msg)

    def control_loop(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.position = False; offboard_msg.velocity = True; offboard_msg.acceleration = False
        self.offboard_mode_pub.publish(offboard_msg)

        setpoint = TrajectorySetpoint()

        if self.offboard_counter < 50:
            self.offboard_counter += 1
            setpoint.velocity = [0.0, 0.0, 0.0]
            self.trajectory_pub.publish(setpoint)
            return

        # ---------------- BARIŞ ANTLAŞMASI (MOD SAVAŞINI BİTİREN KISIM) ----------------
        # Eğer İniş görevi verildiyse, OFFBOARD'a zorlamayı BIRAK!
        if self.durum != "GOREV_TAMAM":
            if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                return
            if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                return

        vx_body, vy_body, vz, yaw_speed = 0.0, 0.0, 0.0, 0.0

        if self.durum == "KALKIS":
            vz = -3.0  
            if self.current_alt >= self.hedef_irtifa - 0.2:
                self.durum = "ARAMA"

        elif self.durum == "ARAMA":
            vz = 0.0
            yaw_speed = 0.4  
            if self.target_found and self.okunan_gorev == "TEKNOFEST_INIS":
                self.kayip_sayaci = 0
                print(f"🎯 INIS GOREVI ONAYLANDI! Hizalanma Süreci Başlıyor...")
                self.durum = "HIZALAN"

        elif self.durum == "HIZALAN":
            if self.target_found:
                self.kayip_sayaci = 0  
                vz = 0.0  
                
                d_err_x = self.err_x - self.prev_err_x
                d_err_y = self.err_y - self.prev_err_y
                
                ham_vy = (self.kp * self.err_x) + (self.kd * d_err_x)
                ham_vx = -((self.kp * self.err_y) + (self.kd * d_err_y))
                
                vy_body = max(-self.max_hiz, min(self.max_hiz, ham_vy))
                vx_body = max(-self.max_hiz, min(self.max_hiz, ham_vx))
                
                self.son_vx_body = vx_body
                self.son_vy_body = vy_body
                self.prev_err_x = self.err_x
                self.prev_err_y = self.err_y
                
                if abs(self.err_x) < self.dinamik_tolerans and abs(self.err_y) < self.dinamik_tolerans:
                    print("✅ HİZALANDI! Süzülme Başlıyor.")
                    self.durum = "ALCAL"
            else:
                self.kayip_sayaci += 1
                if self.kayip_sayaci < 20:  
                    vx_body = self.son_vx_body * 0.9  
                    vy_body = self.son_vy_body * 0.9
                    vz = 0.0
                else:
                    self.durum = "ARAMA"

        elif self.durum == "ALCAL":
            if self.target_found:
                self.kayip_sayaci = 0
                
                if abs(self.err_x) > self.dinamik_tolerans + 40 or abs(self.err_y) > self.dinamik_tolerans + 40:
                    self.durum = "HIZALAN"
                else:
                    vy_body = self.kp * self.err_x * 0.4
                    vx_body = -self.kp * self.err_y * 0.4
                    
                    self.son_vx_body = vx_body
                    self.son_vy_body = vy_body
                    vz = 0.5  
                    
                    # Eğer QR kodu çok yakından gördüysek (Alan büyüdüyse) direkt İN!
                    if self.current_alt <= self.inis_irtifasi or self.hedef_alani > 150000:
                        self.durum = "GOREV_TAMAM"
            else:
                self.kayip_sayaci += 1
                if self.kayip_sayaci < 20:
                    vx_body = self.son_vx_body
                    vy_body = self.son_vy_body
                    vz = 0.0  
                else:
                    self.durum = "ARAMA"

        elif self.durum == "GOREV_TAMAM":
            # 1. LAND moduna geç ve öyle kal!
            if self.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                print("🛬 AUTO.LAND AKTİF! Otonom İniş Yapılıyor...")
            
            # 2. Piste değdiği an pervaneleri sustur!
            if self.current_alt <= 0.25 and self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
                print("🛑 PİSTE TEMAS EDİLDİ! Motorlar başarıyla durduruldu.")
            return # ÖNEMLİ: Hız setpointlerini göndermeyi bırak!

        # Rotasyon Matrisi
        yaw = self.current_yaw
        vx_ned = (vx_body * math.cos(yaw)) - (vy_body * math.sin(yaw))
        vy_ned = (vx_body * math.sin(yaw)) + (vy_body * math.cos(yaw))

        setpoint.velocity = [float(vx_ned), float(vy_ned), float(vz)]
        setpoint.yawspeed = float(yaw_speed)
        setpoint.position = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.yaw = float('nan')
        self.trajectory_pub.publish(setpoint)

def main(args=None):
    rclpy.init(args=args)
    node = TeknofestAvcisi()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()