import cv2
import webbrowser
from pyzbar import pyzbar

def decode_qr_code(frame):
    # pyzbar로 QR 코드 인식
    qr_codes = pyzbar.decode(frame)
    for qr_code in qr_codes:
        qr_data = qr_code.data.decode("utf-8")
        (x, y, w, h) = qr_code.rect
        # QR 코드 주위에 사각형 그리기
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # QR 코드 데이터 출력
        cv2.putText(frame, qr_data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return qr_data
    return None

def main():
    # 웹캠 열기
    cap = cv2.VideoCapture(1)
    detected_urls = set()  # 중복 URL 접속 방지를 위한 집합

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # QR 코드 디코드
        url = decode_qr_code(frame)

        # URL이 유효하고 이전에 접속하지 않았으면 접속
        if url and url not in detected_urls:
            detected_urls.add(url)
            print(f"Detected URL: {url}")
            webbrowser.open(url)  # 웹 브라우저로 URL 열기

        # 결과 출력
        cv2.imshow("QR Code Scanner", frame)

        # 'q'를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
