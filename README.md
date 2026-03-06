#  Teknofest İHA Sürü Otonomisi - Dağıtık Hibrit Lider-Takipçi Sistemi

Bu depo, Teknofest Sürü İHA yarışması için geliştirilmiş **ROS 2 (Humble)** ve **PX4 Autopilot** tabanlı, tam otonom ve dağıtık mimarili bir sürü drone projesini içermektedir.

Sistem; kamera üzerinden yerdeki devasa QR kodları okuyarak otonom karar alabilen, havada dinamik formasyonlar (V, Çizgi, Okbaşı) kurabilen ve görev bittiğinde kalktığı noktaya dönebilen (RTL) algoritmalarla donatılmıştır.

---

## Kullanılan Temel Algoritmalar ve Mimari

Projemiz, işlemci yükünü dağıtmak ve hata toleransını maksimize etmek için **Dağıtık Düğümlü Hibrit Lider-Takipçi** mimarisi üzerine inşa edilmiştir. Lider İHA rotayı belirlerken, takipçi İHA'lar kendi kararlarını kendileri verir.

* **Çarpışma Önleme (Yapay Potansiyel Alanlar - APF):** Boids algoritmasının "Ayrılma" ilkesiyle birleştirilmiş bu sistemde, İHA'lar birbirlerine 2 metreden fazla yaklaştıklarında ters yönde bir kaçış hızı üreterek (Sanal Mıknatıs Kalkanı) çarpışmaları engeller.
* **İşbirlikçi Navigasyon (Sanal Çapa):** Takipçi İHA'lar, Liderin anlık konumunu DDS ağı üzerinden dinleyerek kendi konumlarını 4 metrelik ofsetlerle günceller.
* **Yörünge ve Hizalama Kontrolü (PD & Yaw):** Hedefe uçuşlarda "Balonlama" etkisini önlemek için P-Kontrolcü kullanılmış, yengeç uçuşunu engellemek için ise araç burnu Arktanjant (atan2) ile hedefe kilitlenmiştir.



---

## 📂 Dosya Yapısı

* `tam_suru_ajani.py`: İHA'ların otonom uçuş kaslarıdır. Formasyon, çarpışma önleme, hız hesaplama ve PX4 ile haberleşme bu düğümde gerçekleşir.
* `qr_kamera_beyni.py`: Liderin gözüdür. Gazebo'dan gelen kamera verisini işler, QR kodları (pyzbar) okur ve içindeki JSON görevlerini ROS 2 ağına yayar.
* `sdf_uretici.py` & `arena_qr_uretici.py`: Simülasyona serilecek 15x15 metrelik devasa, zaman ayarlı (10 sn bekleme/hovering) QR kod zeminlerini (SDF formatında) dinamik olarak üretir.

---

##  Kurulum Gereksinimleri

Bu projeyi çalıştırmak için sisteminizde aşağıdakilerin kurulu olması gerekmektedir:
* **İşletim Sistemi:** Ubuntu 22.04 LTS
* **ROS 2:** Humble Hawksbill
* **Simülasyon:** Gazebo (Garden veya Harmonic)
* **Otopilot:** PX4 Autopilot (SITL)
* **DDS:** MicroXRCEAgent
* **Python Kütüphaneleri:** `pip install qrcode pyzbar opencv-python numpy`

---

## 🚀 Uçuş Protokolü (Nasıl Çalıştırılır?)

Aşağıdaki adımları sırayla, ayrı terminallerde çalıştırarak devasa operasyonu başlatabilirsiniz.

### 1. Hazırlık ve Mühimmat Üretimi
İlk olarak workspace'i derleyin ve yerdeki hedef halılarını (SDF) oluşturun:
```bash
cd ~/dds_ws
colcon build
source install/setup.bash
python3 sdf_uretici.py
2. Dünyayı ve Dronları Yaratma (PX4 SITL)
Lider ve 2 Yancıyı Gazebo'da başlatın:

Bash
# Terminal 1 (Lider)
cd ~/PX4-Autopilot
PX4_GZ_MODEL_POSE="0,0" PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_mono_cam ./build/px4_sitl_default/bin/px4 -i 1

# Terminal 2 (Yancı 1)
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_POSE="0,2" PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_mono_cam ./build/px4_sitl_default/bin/px4 -i 2

# Terminal 3 (Yancı 2)
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_POSE="0,-2" PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_mono_cam ./build/px4_sitl_default/bin/px4 -i 3
(Opsiyonel: Dronları QGroundControl üzerinden haritanın rastgele köşelerine uçurup dağıtabilirsiniz. Toplanma algoritması onları havada buluşturacaktır).

3. Hedefleri (QR Kodları) Sahaya Serme
Gazebo açıkken yepyeni bir terminalde sırayla:

Bash
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/kullanici_adi/dds_ws/qr_1.sdf", name: "qr_1", pose: {position: {x: 0.0, y: 0.0, z: 0.005}}'

gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/kullanici_adi/dds_ws/qr_4.sdf", name: "qr_4", pose: {position: {x: 40.0, y: -20.0, z: 0.005}}'

gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/kullanici_adi/dds_ws/qr_2.sdf", name: "qr_2", pose: {position: {x: 80.0, y: 20.0, z: 0.005}}'
4. Haberleşme Köprüleri (DDS & ROS 2)
Bash
# Terminal 4 (MicroDDS)
MicroXRCEAgent udp4 -p 8888

# Terminal 5 (Kamera Köprüsü)
ros2 run ros_gz_bridge parameter_bridge /world/default/model/x500_mono_cam_1/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image --ros-args -r /world/default/model/x500_mono_cam_1/link/camera_link/sensor/camera/image:=/px4_1/camera/image_raw
5. Otonom Kasları Çalıştırma (Toplanma Başlıyor!)
Aşağıdaki komutları girdiğinizde dronlar 20 metreye fırlayıp havada formasyon kuracaktır:

Bash
# Terminal 6, 7 ve 8'de sırayla (source install/setup.bash yapmayı unutmayın):
ros2 run px4_kontrol tam_suru_ajani --ros-args -p ajan_id:=1 -p rol:=LIDER -p takim_id:="team_1"
ros2 run px4_kontrol tam_suru_ajani --ros-args -p ajan_id:=2 -p rol:=YANCI -p takim_id:="team_1"
ros2 run px4_kontrol tam_suru_ajani --ros-args -p ajan_id:=3 -p rol:=YANCI -p takim_id:="team_1"
6. Beyni Ateşleme (Görev Başlıyor!)
Dronlar havada Liderin etrafında V formasyonunu (4 Metre ofsetli) kurduktan sonra beyni çalıştırın:

Bash
# Terminal 9
ros2 run px4_kontrol qr_kamera_beyni
Not: Sistem QGC simülasyon pilini (SIM_BAT_DRAIN) kapalı gerektirir. Failsafe yememek için QGC ayarlarından simüle batarya tüketimini kapatmayı unutmayın.
