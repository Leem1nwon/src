import serial
import time
from pynput import keyboard

# 시리얼 포트 설정 (포트 이름은 자신의 환경에 맞게 수정)
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # Arduino와 통신 준비 시간을 위한 딜레이

# 플래그 변수
forward = False
left = False
right = False
back = False
fast_forward= False

# 키보드 입력이 들어오면 해당 명령을 시리얼로 전송
def on_press(key):
    global forward, left, right, back, fast_forward

    try:
        if key.char == 'w':
            forward = True
            back= False
            update_movement()  # 상태 업데이트
        if key.char == 'q':
            fast_forward = True
            forward=False
            back= False
            update_movement()  # 상태 업데이트
        elif key.char == 'a':
            left = True
            update_movement()  # 상태 업데이트
        elif key.char == 'd':
            right = True
            update_movement()  # 상태 업데이트
        elif key.char == 'x':
            back= True
            forward=False
            update_movement()
        elif key.char == 's':
            forward = False
            left = False
            right = False
            back = False
            ser.write(b'S')  # 멈춤
            print("Stop")
    except AttributeError:
        # 특수 키는 처리하지 않음
        pass

# 키보드에서 키가 떼어지면 상태 업데이트
def on_release(key):
    global forward, left, right, back, fast_forward

    try:
        if key.char == 'w':
            forward = False
            update_movement()  # 상태 업데이트
        if key.char == 'q':
            fast_forward = False
            update_movement()  # 상태 업데이트
        elif key.char == 'a':
            left = False
            update_movement()  # 상태 업데이트
        elif key.char == 'd':
            right = False
            update_movement()  # 상태 업데이트
        elif key.char == 'x':
            back=False
            update_movement()

    except AttributeError:
        pass

    if key == keyboard.Key.esc:
        # ESC를 누르면 종료
        return False

# 상태에 따라 적절한 동작을 Arduino로 전송
def update_movement():
    if forward and left:
        ser.write(b'F')  # 왼쪽으로 전진 (약한 조향)
        print("Forward Left")
    elif forward and right:
        ser.write(b'G')  # 오른쪽으로 전진 (약한 조향)
        print("Forward Right")
    elif forward:
        ser.write(b'W')  # 전진
        print("Forward")
    elif fast_forward and left:
        ser.write(b'B')  # 왼쪽으로 전진 (약한 조향)
        print("FAST Left")
    elif fast_forward and right:
        ser.write(b'N')  # 오른쪽으로 전진 (약한 조향)
        print("FAST Right")
    elif fast_forward:
        ser.write(b'M')  # 전진
        print("FAST")
    elif back and left:
        ser.write(b'H')  # 왼쪽으로 전진 (약한 조향)
        print("Back Left")
    elif back and right:
        ser.write(b'I')  # 오른쪽으로 전진 (약한 조향)
        print("Back Right")
    elif back:
        ser.write(b'X')  # 전진
        print("Back")
    elif left:
        ser.write(b'A')  # 왼쪽 조향
        print("Left")
    elif right:
        ser.write(b'D')  # 오른쪽 조향
        print("Right")
    else:
        ser.write(b'Z')  # 조향각 0도로 복귀
        print("Steering to 0")

# 키보드 입력 리스너 설정
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

# 종료 시 시리얼 포트 닫기
ser.close()
