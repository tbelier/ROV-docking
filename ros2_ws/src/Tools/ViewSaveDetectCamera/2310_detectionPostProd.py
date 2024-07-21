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
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], sizeMarker, camera_matrix, dist_coeffs)

            # DISTANCE
            distance = np.linalg.norm(tvecs[0])
            cv2.putText(frame, f"Distance: {distance:.2f} m", (10, 30),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            #ORIENTATION
            rotation_matrix, _ = cv2.Rodrigues(rvecs[0])
            angles_rad = cv2.RQDecomp3x3(rotation_matrix)[0]
            angles_deg = np.degrees(angles_rad)*180/10000
            cv2.putText(frame, f"Angle: {angles_deg[0]:.0f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"{angles_deg[1]:.0f}", (150, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"{angles_deg[2]:.0f} deg", (210, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return frame


def filter_median(values, new_value):
    values.append(new_value)  # Ajouter la nouvelle valeur à la liste

    if len(values) < 10:
        return values[0]

    else:

        average =np.median(np.array(values))
        values.pop(0)

        return average




if __name__ == "__main__":
    sizeMarker = 0.164 #Length of a size of the Aruco (only white square not the black one around)
    # cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture("saves/airEauDetect.avi")
    # cap = cv2.VideoCapture('saves/airEauDetect.avi')
    #cap = cv2.VideoCapture('saves/TestPourPostprod.webm') #ATTENTION A SA TAILLE ELLE EST DIFFERENTE (1080x1920)

    # camera_matrix = np.array([[750, 0, 300], [0, 750, 240], [0, 0, 1]], dtype=np.float32)  # Matrice de la caméra
    # dist_coeffs = np.array([[ 2.00172236e-01],[-1.03623068e+00],[-8.67720681e-03],[-1.46253428e-03],[3.15958548e+00]],  dtype=np.float32)

    camera_matrix = np.array([[702, 0, 617], [0, 698, 393], [0, 0, 1]], dtype=np.float32)  # Matrice de la caméra
    dist_coeffs = np.array([[0.06991307, -0.10688273, -0.00414977, 0.01193672, 0.09424899]], dtype=np.float32)


    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
    resize_coeff = 0.8
    #HOMOGENISATION DES CONTRASTES

    while True:
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        height, width, _ = frame.shape
        cv2.imshow('Gray cam', cv2.resize(gray, (int(width * resize_coeff), int(height * resize_coeff))))


        """
        without_ani_frame = gray
        detect(without_ani_frame, sizeMarker)
        cv2.imshow('ROVcam Aruco Detection', cv2.resize(without_ani_frame, (int(width * resize_coeff), int(height * resize_coeff))))
        """

        """
        with_ani_frame = cv2.ximgproc.anisotropicDiffusion(frame, 0.1, 4, 3)
        gray_with_ani = cv2.cvtColor(with_ani_frame, cv2.COLOR_BGR2GRAY)
        detect(gray_with_ani, sizeMarker)
        cv2.imshow('Modified', cv2.resize(gray_with_ani, (int(width * resize_coeff), int(height * resize_coeff))))
        """

        kernel = np.array([[-1, -1, -1],
                           [-1, 9, -1],
                           [-1, -1, -1]])

        gamma = 1.5
        gamma_corrected = np.power(frame / 255.0, 1.0 / gamma)
        gamma_corrected = (gamma_corrected * 255).astype('uint8')

        sharpened_image = cv2.filter2D(gamma_corrected, -1, kernel)
        filtered_image = cv2.ximgproc.anisotropicDiffusion(sharpened_image, 0.5, 5, 1)
        detected_image = detect(filtered_image, sizeMarker)

        cv2.imshow('Kernel', cv2.resize(detected_image, (int(width * resize_coeff), int(height * resize_coeff))))

        # STOP IF Q PRESSED
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()