import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class RenkAvcisi(Node):
    def __init__(self):
        super().__init__('renk_avcisi_node')
        self.topic_name = '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image'
        self.subscription = self.create_subscription(Image, self.topic_name, self.listener_callback, qos_profile_sensor_data)
        self.br = CvBridge()
        print("🚁 Avcı Drone Aktif! KIRMIZI hedef aranıyor...")

    def listener_callback(self, data):
        try:
            # 1. Görüntüyü Al
            frame = self.br.imgmsg_to_cv2(data, "bgr8")
            
            # 2. Renk algılamayı kolaylaştırmak için BGR'den HSV formatına çevir
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # 3. Kırmızı renk için sınırları belirle (OpenCV'de kırmızı iki bölgededir)
            alt_kirmizi1 = np.array([0, 100, 100])
            ust_kirmizi1 = np.array([10, 255, 255])
            alt_kirmizi2 = np.array([160, 100, 100])
            ust_kirmizi2 = np.array([179, 255, 255])
            
            # 4. Kırmızı olan yerleri beyaz, diğer her yeri siyah yapan bir maske oluştur
            maske1 = cv2.inRange(hsv, alt_kirmizi1, ust_kirmizi1)
            maske2 = cv2.inRange(hsv, alt_kirmizi2, ust_kirmizi2)
            tam_maske = maske1 + maske2
            
            # 5. Beyaz bölgelerin sınırlarını (konturlarını) bul
            konturlar, _ = cv2.findContours(tam_maske, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Eğer ekranda kırmızı bir şey varsa:
            if konturlar:
                # En büyük kırmızı alanı seç (Ufak tefek kırmızı pikselleri yoksay)
                en_buyuk_kontur = max(konturlar, key=cv2.contourArea)
                
                # Alan yeterince büyükse hedef olarak kabul et
                if cv2.contourArea(en_buyuk_kontur) > 500:
                    x, y, w, h = cv2.boundingRect(en_buyuk_kontur)
                    merkez_x = x + (w // 2)
                    merkez_y = y + (h // 2)
                    
                    # Hedefin etrafına yeşil kutu çiz ve merkezine nokta koy
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                    cv2.circle(frame, (merkez_x, merkez_y), 5, (255, 0, 0), -1)
                    
                    print(f"🎯 KİLİTLENDİ! Hedefin Ekrandaki Konumu: X={merkez_x}, Y={merkez_y}")
            
            # Canlı yayını göster
            cv2.imshow("Avci Drone KamerasI", frame)
            cv2.waitKey(1)
            
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = RenkAvcisi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()