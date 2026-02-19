#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode, VehicleLocalPosition
import math

class KareDevriye(Node):
    def __init__(self):
        super().__init__('devriye_node')

        # --- QOS AYARLARI ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- YAYINCILAR ---
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)

        # --- ABONE ---
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1', 
            self.position_callback,
            qos_profile
        )

        # --- DEĞİŞKENLER ---
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0
        self.current_position = [0.0, 0.0, 0.0]
        
        # --- DURUM KONTROLÜ ---
        self.inis_basladi = False
        
        # --- ROTA ---
        self.waypoints = [
            [0.0, 0.0, -5.0],  # 0. Kalkış
            [5.0, 0.0, -5.0],  # 1. Kuzey
            [5.0, 5.0, -5.0],  # 2. Doğu
            [0.0, 5.0, -5.0],  # 3. Geri
            [0.0, 0.0, -5.0],  # 4. Merkez (Buraya gelince İNİŞ moduna geçeceğiz)
        ]
        self.current_wp_index = 0
        self.hata_payi = 0.3

    def position_callback(self, msg):
        self.current_position = [msg.x, msg.y, msg.z]

    def timer_callback(self):
        # Eğer iniş komutu verildiyse artık Offboard sinyali gönderme, 
        # PX4 Land moduna geçsin ve işi bitirsin.
        if self.inis_basladi:
            return

        # Uçuş sırasında Offboard sinyalini canlı tut
        self.publish_offboard_control_mode()

        # --- KALKIŞ (ARM) ---
        if self.counter == 15:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info("🚀 Motorlar Aktif! Görev Başlıyor...")

        # --- ROTA TAKİBİ ---
        if self.counter > 15:
            hedef = self.waypoints[self.current_wp_index]
            self.publish_trajectory_setpoint(hedef)
            
            dx = hedef[0] - self.current_position[0]
            dy = hedef[1] - self.current_position[1]
            dz = hedef[2] - self.current_position[2]
            mesafe = math.sqrt(dx*dx + dy*dy + dz*dz)

            # Kalkış Kontrolü
            if self.current_wp_index == 0 and self.current_position[2] > -1.0:
                 if self.counter % 20 == 0:
                     self.get_logger().info(f"⏳ Yükselme Bekleniyor... Z: {self.current_position[2]:.2f}")
            
            # Hedefe Vardık mı?
            elif mesafe < self.hata_payi:
                # Son hedef değilse sonrakine geç
                if self.current_wp_index < len(self.waypoints) - 1:
                    self.get_logger().info(f"✅ Hedef {self.current_wp_index} Tamamlandı. Sıradaki...")
                    self.current_wp_index += 1
                
                # SON HEDEF (Merkez Üstü) TAMAMLANDI!
                else:
                    self.get_logger().info("🏁 MERKEZE DÖNÜLDÜ. İniş Moduna Geçiliyor (Land Mode)...")
                    
                    # --- KRİTİK HAMLE: LAND KOMUTU ---
                    # Manuel yazdığın 'commander land' komutunun aynısıdır.
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    
                    self.inis_basladi = True # Kodun müdahalesini kes

        self.counter += 1

    # --- YARDIMCI FONKSİYONLAR ---
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, hedef):
        msg = TrajectorySetpoint()
        msg.position = hedef
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KareDevriye()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()