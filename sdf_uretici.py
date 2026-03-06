import os

print("🏭 SDF Harita Modelleri Üretiliyor...")

modeller = [
    {"id": 1, "resim": "qr_1.png"},
    {"id": 4, "resim": "qr_4.png"},
    {"id": 2, "resim": "qr_2.png"}
]

for model in modeller:
    sdf_icerik = f"""<sdf version="1.9">
  <model name="qr_{model['id']}_zemin">
    <static>true</static>
    <link name="base_link">
      <visual name="visual">
        <geometry><plane><normal>0 0 1</normal><size>15.0 15.0</size></plane></geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
          <pbr><metal>
            <albedo_map>file:///home/enesturkoglu2/dds_ws/{model['resim']}</albedo_map>
            <roughness>1.0</roughness><metalness>0.0</metalness>
          </metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
    
    dosya_adi = f"qr_{model['id']}.sdf"
    with open(dosya_adi, "w") as f:
        f.write(sdf_icerik)
    print(f"✅ {dosya_adi} başarıyla oluşturuldu!")

print("🚀 Tüm SDF modelleri hazır! Artık Gazebo'ya fırlatabiliriz.")