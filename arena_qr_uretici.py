import qrcode
import json

senaryo = [
    {
        "dosya_adi": "qr_1.png",
        "veri": {
            "qr_id": 1,
            # 📌 Lider burada 10 SANİYE bekleyecek!
            "gorev": {"formasyon": {"aktif": True, "tip": "OKBASI"}, "irtifa_degisim": {"aktif": True, "deger": 20}, "bekleme_suresi_s": 10},
            "sonraki_qr": {"team_1": 4}
        }
    },
    {
        "dosya_adi": "qr_4.png",
        "veri": {
            "qr_id": 4,
            # 📌 Lider burada 10 SANİYE bekleyecek!
            "gorev": {"formasyon": {"aktif": True, "tip": "CIZGI"}, "irtifa_degisim": {"aktif": True, "deger": 15}, "bekleme_suresi_s": 10},
            "sonraki_qr": {"team_1": 2}
        }
    },
    {
        "dosya_adi": "qr_2.png",
        "veri": {
            "qr_id": 2,
            # 📌 0 komutu geldiğinde 5 saniye bekleyip EVE DÖNECEKLER!
            "gorev": {"formasyon": {"aktif": True, "tip": "V"}, "irtifa_degisim": {"aktif": True, "deger": 10}, "bekleme_suresi_s": 5},
            "sonraki_qr": {"team_1": 0} 
        }
    }
]

for gorev in senaryo:
    qr = qrcode.QRCode(version=1, box_size=20, border=2)
    qr.add_data(json.dumps(gorev["veri"], indent=2))
    qr.make(fit=True)
    resim = qr.make_image(fill_color="black", back_color="white")
    resim.save(gorev["dosya_adi"])

print("✅ Yeni zaman ayarlı mühimmatlar hazır!")