#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# PX4'ün "Kuş Dili" mesajlarını ekliyoruz
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode

class YukselenKartal(Node):
    def __init__(self):
        super().__init__('kalkis_node')

        # --- 1. QOS AYARLARI (Çok Önemli!) ---
        # DDS hızlı veri ister, o yüzden "Best Effort" kullanıyoruz.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- 2. YAYINCILAR (Publisher) ---
        # Drone'a "Şuraya Git" demek için:
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        # Drone'a "Mod Değiştir / Arm Ol" demek için:
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        # Drone'a "Ben Bilgisayarım, Kontrol Bende" demek için:
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)

        # --- 3. ZAMANLAYICI (Heartbeat) ---
        # Saniyede 10 kere (0.1s) bu fonksiyonu çalıştır.
        # PX4, 0.5 saniye sesimizi duymazsa kontrolü bırakır ve yere çakılır!
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0

        self.get_logger().info("🦅 Yükselen Kartal Hazır! Kalkışa hazırlanılıyor...")

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # İlk 10 döngü (1 saniye) sadece sinyal gönderiyoruz ki PX4 bizi fark etsin.
        # Sonra "Offboard" moduna geçip motorları (Arm) açıyoruz.
        if self.counter == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # Offboard Modu
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0) # Arm (Motorları Aç)
            self.get_logger().info("🚀 Motorlar Çalıştı! 5 Metreye Yükseliyoruz!")

        if self.counter < 11:
            self.counter += 1

    # --- YARDIMCI FONKSİYONLAR ---

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True  # Sadece Konum kontrolü yapacağız
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        
        # --- NED KOORDİNAT SİSTEMİ ---
        # X: Kuzey (+), Güney (-)
        # Y: Doğu (+), Batı (-)
        # Z: Aşağı (+), Yukarı (-)  <-- DİKKAT! Yukarı çıkmak için Eksi veriyoruz.
        
        msg.position = [0.0, 0.0, -5.0] # 5 Metre YUKARI
        msg.yaw = 0.0 # 0 derece (Kuzeye bak)
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
    node = YukselenKartal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()