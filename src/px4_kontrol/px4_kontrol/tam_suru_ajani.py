import rclpy
from rclpy.node import Node
import math
import json
from std_msgs.msg import String
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from geometry_msgs.msg import Point  
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class TamSuruAjani(Node):
    def __init__(self):
        super().__init__('tam_suru_ajani_node')
        
        self.declare_parameter('ajan_id', 1) 
        self.declare_parameter('rol', 'LIDER')
        self.declare_parameter('takim_id', 'team_1')
        
        self.ajan_id = self.get_parameter('ajan_id').get_parameter_value().integer_value
        self.rol = self.get_parameter('rol').get_parameter_value().string_value
        self.takim_id = self.get_parameter('takim_id').get_parameter_value().string_value
        
        self.dogum_x = 0.0
        self.dogum_y = 0.0
        if self.ajan_id == 2: self.dogum_y = 2.0  
        if self.ajan_id == 3: self.dogum_y = -2.0 

        self.form_x = 0.0
        self.form_y = 0.0
        self.formasyon_ayarla("V") 
        
        px4_ns = f'/px4_{self.ajan_id}'
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        self.durum = "KALKIS" 
        
        self.kendi_local_x = 0.0; self.kendi_local_y = 0.0; self.kendi_alt = 0.0
        self.ortak_x = 0.0; self.ortak_y = 0.0 
        self.nav_state = 0; self.arm_state = 0

        self.kp_xy = 0.6  
        self.kp_z = 1.0   
        self.max_hiz = 3.0

        self.gorev_abonesi = self.create_subscription(String, '/suru_gorev_komutlari', self.gorev_callback, 10)
        
        self.qr_haritasi = {1: (0.0, 0.0), 2: (80.0, 20.0), 3: (-10.0, 15.0), 4: (40.0, -20.0), 5: (0.0, 20.0)}
        
        self.asil_hedef_x = 0.0
        self.asil_hedef_y = 0.0
        self.hedef_z = 20.0 
        self.hedef_yaw = 0.0 
        self.son_okunan_qr = None

        self.bekleme_suresi = 0.0
        self.bekleme_baslangic = 0.0
        self.toplanma_baslangic = 0.0
        self.siradaki_hedef_x = 0.0
        self.siradaki_hedef_y = 0.0
        self.eve_donus_aktif = False

        self.suru_yayin = self.create_publisher(Point, f'/suru_haberlesme/ajan_{self.ajan_id}', qos)
        self.suru_haritasi = {1: None, 2: None, 3: None} 
        for i in range(1, 4):
            if i != self.ajan_id:
                self.create_subscription(Point, f'/suru_haberlesme/ajan_{i}', lambda msg, d_id=i: self.sistem_telsizi_cb(msg, d_id), qos)

        self.pose_sub = self.create_subscription(VehicleLocalPosition, f'{px4_ns}/fmu/out/vehicle_local_position_v1', self.pose_cb, qos)
        self.status_sub = self.create_subscription(VehicleStatus, f'{px4_ns}/fmu/out/vehicle_status_v1', self.status_cb, qos)
        self.mode_pub = self.create_publisher(OffboardControlMode, f'{px4_ns}/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, f'{px4_ns}/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, f'{px4_ns}/fmu/in/vehicle_command', 10)

        self.sayac = 0
        self.timer = self.create_timer(0.05, self.sistem_dongusu) 
        print(f"🚀 AJAN UYANDI | ID: {self.ajan_id} | ROL: {self.rol}")

    def gorev_callback(self, msg):
        try:
            qr_verisi = json.loads(msg.data)
            su_anki_qr_id = qr_verisi.get("qr_id")

            if su_anki_qr_id == self.son_okunan_qr: return
            self.son_okunan_qr = su_anki_qr_id

            print(f"\n[{self.rol} {self.ajan_id}] 📻 GÖREV ALINDI! (QR {su_anki_qr_id})")

            gorev_paketi = qr_verisi.get("gorev", {})
            self.bekleme_suresi = float(gorev_paketi.get("bekleme_suresi_s", 0.0))
            self.bekleme_baslangic = self.get_clock().now().nanoseconds / 1e9

            irtifa_verisi = gorev_paketi.get("irtifa_degisim", {})
            if irtifa_verisi.get("aktif"): self.hedef_z = float(irtifa_verisi.get("deger"))

            formasyon_verisi = gorev_paketi.get("formasyon", {})
            if formasyon_verisi.get("aktif"): self.formasyon_ayarla(formasyon_verisi.get("tip"))

            benim_rotam = qr_verisi.get("sonraki_qr", {}).get(self.takim_id)
            if benim_rotam in self.qr_haritasi:
                koordinat = self.qr_haritasi[benim_rotam]
                self.siradaki_hedef_x = koordinat[0]
                self.siradaki_hedef_y = koordinat[1]
                self.durum = "GOREV_BEKLEME"
                print(f"⏳ {self.bekleme_suresi} Saniye Formasyon Şovu! Sonraki İstikamet: QR {benim_rotam}")
            
            elif benim_rotam == 0:
                self.eve_donus_aktif = True
                self.durum = "GOREV_BEKLEME"
                print(f"⏳ Görev Bitti! {self.bekleme_suresi} Saniye Şov Yapılıp EVE DÖNÜLECEK (RTL)!")
                
        except json.JSONDecodeError: pass

    def formasyon_ayarla(self, tip):
        # 📌 SİHİRLİ DOKUNUŞ: FORMASYON 4 METREYE GENİŞLETİLDİ! (Balonlama ve Türbülansı Önlemek İçin)
        if tip == "OKBASI":
            if self.rol != 'LIDER': self.form_x, self.form_y = -3.0, (4.0 if self.ajan_id==2 else -4.0)
        elif tip == "CIZGI":
            if self.rol != 'LIDER': self.form_x, self.form_y = 0.0, (4.0 if self.ajan_id==2 else -4.0)
        elif tip == "V":
            if self.rol != 'LIDER': self.form_x, self.form_y = 3.0, (4.0 if self.ajan_id==2 else -4.0)

    def sistem_telsizi_cb(self, msg, d_id): self.suru_haritasi[d_id] = [msg.x, msg.y, msg.z]

    def pose_cb(self, msg): 
        self.kendi_local_x = msg.y  
        self.kendi_local_y = msg.x  
        self.kendi_alt = -msg.z     
        self.ortak_x = self.kendi_local_x + self.dogum_x
        self.ortak_y = self.kendi_local_y + self.dogum_y

    def status_cb(self, msg): self.nav_state = msg.nav_state; self.arm_state = msg.arming_state

    def komut_gonder(self, komut, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = komut; msg.param1 = float(p1); msg.param2 = float(p2)
        msg.target_system = self.ajan_id + 1  
        msg.target_component = 1; msg.source_system = 255; msg.source_component = 1; msg.from_external = True
        self.cmd_pub.publish(msg)

    def setpoint_hiz(self, vx, vy, vz_down, yaw):
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        sp.position = [float('nan'), float('nan'), float('nan')] 
        sp.velocity = [float(vx), float(vy), float(vz_down)] 
        sp.yaw = float(yaw)
        return sp

    def sistem_dongusu(self):
        if self.durum == "INIS": return

        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        self.mode_pub.publish(offboard_msg)

        yayin = Point(); yayin.x = self.ortak_x; yayin.y = self.ortak_y; yayin.z = self.kendi_alt
        self.suru_yayin.publish(yayin)

        anlik_hedef_x = self.ortak_x
        anlik_hedef_y = self.ortak_y

        if self.durum == "KALKIS":
            if self.kendi_alt > 19.5:
                if self.rol == 'LIDER':
                    self.asil_hedef_x = self.ortak_x
                    self.asil_hedef_y = self.ortak_y
                self.durum = "TOPLANMA"
                self.toplanma_baslangic = self.get_clock().now().nanoseconds / 1e9
                print(f"✅ {self.rol} 20 Metreye Ulaştı! Liderin Etrafında Toplanılıyor...")

        elif self.durum == "TOPLANMA":
            if self.rol == 'LIDER':
                anlik_hedef_x = self.asil_hedef_x
                anlik_hedef_y = self.asil_hedef_y
            else:
                lider_konum = self.suru_haritasi[1]
                if lider_konum is not None:
                    anlik_hedef_x = lider_konum[0] + self.form_x
                    anlik_hedef_y = lider_konum[1] + self.form_y

            su_an = self.get_clock().now().nanoseconds / 1e9
            if su_an - self.toplanma_baslangic > 15.0:
                if self.rol == 'LIDER': print("🚀 TOPLANMA TAMAMLANDI! FİLO QR 1'E (MERKEZE) AKIYOR!")
                self.durum = "MERKEZE_INTIKAL"

        elif self.durum == "MERKEZE_INTIKAL":
            if self.rol == 'LIDER':
                anlik_hedef_x = 0.0
                anlik_hedef_y = 0.0
                mesafe_merkez = math.sqrt(self.ortak_x**2 + self.ortak_y**2)
                if mesafe_merkez < 1.0:
                    self.durum = "BEKLEME_ILK"
            else:
                lider_konum = self.suru_haritasi[1]
                if lider_konum is not None:
                    anlik_hedef_x = lider_konum[0] + self.form_x
                    anlik_hedef_y = lider_konum[1] + self.form_y
                    mesafe_lider_merkez = math.sqrt(lider_konum[0]**2 + lider_konum[1]**2)
                    if mesafe_lider_merkez < 1.0:
                        self.durum = "BEKLEME_ILK"

        elif self.durum == "BEKLEME_ILK":
            if self.rol == 'LIDER':
                anlik_hedef_x = 0.0
                anlik_hedef_y = 0.0
            else:
                lider_konum = self.suru_haritasi[1]
                if lider_konum is not None:
                    anlik_hedef_x = lider_konum[0] + self.form_x
                    anlik_hedef_y = lider_konum[1] + self.form_y

        elif self.durum == "GOREV_BEKLEME":
            if self.rol == 'LIDER':
                anlik_hedef_x = self.asil_hedef_x
                anlik_hedef_y = self.asil_hedef_y
            else:
                lider_konum = self.suru_haritasi[1]
                if lider_konum is not None:
                    anlik_hedef_x = lider_konum[0] + self.form_x
                    anlik_hedef_y = lider_konum[1] + self.form_y

            su_an = self.get_clock().now().nanoseconds / 1e9
            if su_an - self.bekleme_baslangic >= self.bekleme_suresi:
                if self.eve_donus_aktif:
                    self.asil_hedef_x = 0.0 
                    self.asil_hedef_y = 0.0 
                    self.durum = "EVE_DON"
                    print("🚀 ŞOV BİTTİ! FİLO ÜSSE DÖNÜYOR (RTL)...")
                else:
                    self.asil_hedef_x = self.siradaki_hedef_x
                    self.asil_hedef_y = self.siradaki_hedef_y
                    self.durum = "GOREVE_GIT"
                    print("🚀 ŞOV BİTTİ! YENİ HEDEFE İNTİKAL BAŞLADI!")

        elif self.durum == "GOREVE_GIT" or self.durum == "EVE_DON":
            if self.rol == 'LIDER':
                anlik_hedef_x = self.asil_hedef_x
                anlik_hedef_y = self.asil_hedef_y
            else:
                lider_konum = self.suru_haritasi[1]
                if lider_konum is not None:
                    anlik_hedef_x = lider_konum[0] + self.form_x
                    anlik_hedef_y = lider_konum[1] + self.form_y
            
            if self.durum == "EVE_DON":
                mesafe_ev = math.sqrt(self.ortak_x**2 + self.ortak_y**2)
                # 📌 İNİŞ İÇİN HATA PAYINI GENİŞLETTİK (Yancılar da rahatça insin diye)
                if mesafe_ev < 5.0:
                    print(f"🏁 [{self.rol}] MERKEZ ÜSSE ULAŞTI! İNİŞE GEÇİYOR!")
                    self.durum = "INIS"
                    self.komut_gonder(VehicleCommand.VEHICLE_CMD_NAV_LAND)

        # Burun Hizalama (Yaw)
        mesafe_x = anlik_hedef_x - self.ortak_x
        mesafe_y = anlik_hedef_y - self.ortak_y
        if math.sqrt(mesafe_x**2 + mesafe_y**2) > 0.5:
            self.hedef_yaw = math.atan2(mesafe_x, mesafe_y)

        # Çarpışma Kalkanı
        kacis_vx = 0.0; kacis_vy = 0.0
        for d_id, konum in self.suru_haritasi.items():
            if konum is not None and d_id != self.ajan_id:
                suru_fark_x = self.ortak_x - konum[0]
                suru_fark_y = self.ortak_y - konum[1]
                mesafe = math.sqrt(suru_fark_x**2 + suru_fark_y**2)
                if 0.1 < mesafe < 2.0:
                    itme = 1.0 * (2.0 - mesafe)
                    kacis_vx += (suru_fark_x / mesafe) * itme
                    kacis_vy += (suru_fark_y / mesafe) * itme

        err_x = anlik_hedef_x - self.ortak_x  
        err_y = anlik_hedef_y - self.ortak_y  
        err_z = self.hedef_z - self.kendi_alt

        vx_gazebo = self.kp_xy * err_x + kacis_vx
        vy_gazebo = self.kp_xy * err_y + kacis_vy
        vz_down = -(self.kp_z * err_z) 

        vx_gazebo = max(-self.max_hiz, min(self.max_hiz, vx_gazebo))
        vy_gazebo = max(-self.max_hiz, min(self.max_hiz, vy_gazebo))
        vz_down = max(-2.0, min(2.0, vz_down))

        vx_px4 = vy_gazebo 
        vy_px4 = vx_gazebo 

        self.traj_pub.publish(self.setpoint_hiz(vx_px4, vy_px4, vz_down, self.hedef_yaw))

        self.sayac += 1
        if self.sayac > 20:
            if self.arm_state != 2: self.komut_gonder(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0)
            if self.nav_state != 14: self.komut_gonder(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=6.0)

def main(args=None):
    rclpy.init(args=args)
    node = TamSuruAjani()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()