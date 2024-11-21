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
create_qr_code("A_1", "qr_code_A_1.png")
create_qr_code("A_2", "qr_code_A_2.png")
create_qr_code("A_3", "qr_code_A_3.png")
create_qr_code("B_1", "qr_code_B_1.png")
create_qr_code("B_2", "qr_code_B_2.png")
create_qr_code("B_3", "qr_code_B_3.png")
