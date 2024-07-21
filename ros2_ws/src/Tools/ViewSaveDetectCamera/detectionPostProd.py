import cv2
import cv2.aruco as aruco
import numpy as np
from scipy.signal import convolve2d
import cv2.ximgproc as ximgproc

def detect(frame, sizeMarker, resize_coeff=1):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        for i, marker_id in enumerate(ids):
            # COMPUTING
            print(i, marker_id)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], sizeMarker, camera_matrix, dist_coeffs)

            # DISTANCE
            distance = np.linalg.norm(tvecs[0])
            cv2.putText(frame, f"Distance: {distance:.2f} m", (10, 30),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            #ORIENTATION
            rotation_matrix, _ = cv2.Rodrigues(rvecs[0])
            angles_rad = cv2.RQDecomp3x3(rotation_matrix)[0]
            angles_deg = np.degrees(angles_rad)*180/10000
            cv2.putText(frame, f"Angle: {angles_deg[0]:.0f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(frame, f"{angles_deg[1]:.0f}", (150, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(frame, f"{angles_deg[2]:.0f} deg", (210, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    return frame


def filter_median(values, new_value):
    values.append(new_value)  # Ajouter la nouvelle valeur à la liste

    if len(values) < 10:
        return values[0]

    else:

        average =np.median(np.array(values))
        values.pop(0)

        return average

def image_treatment(frame):
    clahe = cv2.createCLAHE(clipLimit=3., tileGridSize=(10, 10))
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)  # convert from BGR to LAB color space
    l, a, b = cv2.split(lab)  # split on 3 different channels
    l2 = clahe.apply(l)  # apply CLAHE to the L-channel
    lab = cv2.merge((l2, a, b))  # merge channels
    cv_image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)  # convert from LAB to BGR

    gamma = 3#0.7  ##corrige la brightness non linéairement
    lookUpTable = np.array([np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255) for i in range(256)], dtype=np.uint8)
    return cv2.LUT(cv_image, lookUpTable)


if __name__ == "__main__":
    sizeMarker = 0.164
    cap = cv2.VideoCapture("saves/airEauDetect.avi")
    #cap.set(cv2.CAP_PROP_POS_MSEC, 30000)


    camera_matrix = np.array([[702, 0, 617], [0, 698, 393], [0, 0, 1]], dtype=np.float32)  # Matrice de la caméra
    dist_coeffs = np.array([[0.06991307, -0.10688273, -0.00414977, 0.01193672, 0.09424899]], dtype=np.float32)


    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
    resize_coeff = 0.8

    while True:
        ret, frame = cap.read()
        cv2.imshow('init image', frame)
        height, width, _ = frame.shape


        cv_image = image_treatment(frame)

        detected_image = detect(cv_image, sizeMarker)
        cv2.imshow('image', cv_image)

        # STOP IF Q PRESSED
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()