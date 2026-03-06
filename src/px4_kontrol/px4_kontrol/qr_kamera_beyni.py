import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
from pyzbar.pyzbar import decode

class QRAkilliGoz(Node):
    def __init__(self):
        super().__init__('qr_akilli_goz_node')
        
        self.declare_parameter('takim_id', 'team_1') 
        self.takim_id = self.get_parameter('takim_id').get_parameter_value().string_value
        
        self.kamera_sub = self.create_subscription(Image, '/px4_1/camera/image_raw', self.kamera_cb, 10)
        self.komut_yayinlayici = self.create_publisher(String, '/suru_gorev_komutlari', 10)
        
        self.cv_bridge = CvBridge()
        self.son_yayin_zamani = 0.0 # Zamanlayıcı ekledik!
        
        print(f"👁️ RADAR VE 📻 TELSİZ AKTİF! | Hedef: {self.takim_id}")

    def kamera_cb(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            qr_kodlar = decode(cv_image)
            
            if len(qr_kodlar) > 0:
                su_an = self.get_clock().now().nanoseconds / 1e9
                
                # Saniyede 30 kere değil, her 2 saniyede 1 kere telsize bas!
                if su_an - self.son_yayin_zamani > 2.0:
                    veri = qr_kodlar[0].data.decode('utf-8')
                    self.json_parcala_ve_karar_ver(veri)
                    self.son_yayin_zamani = su_an
                    
            cv2.imshow("LIDER KAMERA (Ayar Penceresi)", cv_image)
            cv2.waitKey(1)
        except Exception:
            pass

    def json_parcala_ve_karar_ver(self, qr_metni):
        try:
            bas = qr_metni.find('{')
            son = qr_metni.rfind('}')
            if bas != -1 and son != -1:
                temiz_metin = qr_metni[bas:son+1]
            else:
                return

            qr_verisi = json.loads(temiz_metin)
            
            print(f"🎯 QR {qr_verisi.get('qr_id')} GÖRÜLDÜ -> TELSİZDEN YAYINLANIYOR! 🔊")
            
            # Telsizden Yayınla
            telsiz_mesaji = String()
            telsiz_mesaji.data = temiz_metin
            self.komut_yayinlayici.publish(telsiz_mesaji)

        except Exception as e:
            print("❌ JSON HATA:", e)

def main(args=None):
    rclpy.init(args=args)
    node = QRAkilliGoz()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown(); cv2.destroyAllWindows()

if __name__ == '__main__': main()