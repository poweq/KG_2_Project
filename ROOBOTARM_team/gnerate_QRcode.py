import qrcode

def create_qr_code(data, filename):
    # QR 코드 생성
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_L,
        box_size=10,
        border=4,
    )
    qr.add_data(data)
    qr.make(fit=True)

    # QR 코드 이미지를 만들고 저장
    img = qr.make_image(fill="black", back_color="white")
    img.save(filename)
    print(f"{data}에 해당하는 QR 코드가 생성되었습니다: {filename}")

# QR 코드 생성
create_qr_code("A", "qr_code_A.png")
create_qr_code("B", "qr_code_B.png")
create_qr_code("C", "qr_code_C.png")
