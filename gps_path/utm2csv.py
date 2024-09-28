import csv

# 파일 경로 설정
input_file = "/home/lmw/catkin_ws/src/gps_path/ego_path.txt"  # UTM 좌표가 저장된 입력 파일
output_file = "/home/lmw/catkin_ws/src/gps_path/utm_coordinates.csv"  # 저장할 CSV 파일 경로

# CSV 파일 생성
with open(input_file, 'r') as infile, open(output_file, 'w', newline='') as outfile:
    reader = infile.readlines()
    writer = csv.writer(outfile)

    # CSV 파일에 헤더 추가
    writer.writerow(["Easting", "Northing"])

    # 각 줄을 읽어서 CSV로 작성
    for line in reader:
        easting, northing = line.strip().split()
        writer.writerow([easting, northing])

print(f"CSV 파일이 성공적으로 생성되었습니다: {output_file}")
