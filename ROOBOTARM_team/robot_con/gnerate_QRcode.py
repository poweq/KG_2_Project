import qrcode

# QR 코드 데이터 및 URL 정의
qr_data_to_urls = {
    "A_1": "https://site.naver.com/patient/A_1",
    "A_2": "https://site.naver.com/patient/A_2",
    "A_3": "https://site.naver.com/patient/A_3",
    "B_1": "https://site.naver.com/patient/B_1",
    "B_2": "https://site.naver.com/patient/B_2",
    "B_3": "https://site.naver.com/patient/B_3",
}

# QR 코드 생성 함수
def create_qr_codes(qr_data):
    for key, url in qr_data.items():
        qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_L,
            box_size=10,
            border=4,
        )
        qr.add_data(url)
        qr.make(fit=True)
        qr_image = qr.make_image(fill_color="black", back_color="white")
        # QR 코드 이미지를 파일로 저장
        qr_image.save(f"{key}.png")
        print(f"QR 코드 생성 완료: {key}.png")

# QR 코드 생성 실행
create_qr_codes(qr_data_to_urls)
