import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class KonumKontrol(Node):
    def __init__(self):
        super().__init__('konum_kontrol_node')

        # PX4 için en güvenli QoS ayarı
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Yerel konumu dinliyoruz (X, Y, Z ve Hız)
        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.listener_callback,
            qos_profile
        )
        print("🛰️ Konum Dedektifi Aktif! Veri bekleniyor...")

    def listener_callback(self, msg):
        # Drone'un başlangıç noktasına göre konumu (metre cinsinden)
        print(f"📍 Konum -> X: {msg.x:.2f} | Y: {msg.y:.2f} | Z: {msg.z:.2f} | Hız: {msg.vx:.2f} m/s")

def main(args=None):
    rclpy.init(args=args)
    node = KonumKontrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()