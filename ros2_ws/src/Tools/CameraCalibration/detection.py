import cv2
import cv2.aruco as aruco
import numpy as np

# Initialisation de la capture vidéo depuis la webcam
#cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture('saves_video/test_without_detection_1.avi')
cap = cv2.VideoCapture('saves_video/airEauDetect.avi')
#cap = cv2.VideoCapture('saves_video/TestPourPostprod.webm') #ATTENTION A SA TAILLE ELLE EST DIFFERENTE (1080x1920)

# Paramètres de la caméra (à ajuster selon votre caméra)
camera_matrix = np.array([[750, 0, 300], [0, 750, 240], [0, 0, 1]], dtype=np.float32)  # Matrice de la caméra
dist_coeffs = np.array([[ 2.00172236e-01],[-1.03623068e+00],[-8.67720681e-03],[-1.46253428e-03],[3.15958548e+00]],  dtype=np.float32)
#dist_coeffs = np.zeros((4, 1), dtype=np.float32)  # Coefficients de distorsion

# Création d'un dictionnaire Aruco
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)

while True:
    # Capture d'une image depuis la webcam
    ret, frame = cap.read()

    # Conversion de l'image en niveaux de gris
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Détection des marqueurs Aruco
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)

    # Dessiner les marqueurs détectés sur l'image
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        # Affichage des IDs des marqueurs détectés
        for i, marker_id in enumerate(ids):
            # Calcul de la pose (position et orientation) du marqueur
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.05, camera_matrix, dist_coeffs)

            # Calcul de la distance entre la caméra et le marqueur
            distance = np.linalg.norm(tvecs[0])
            cv2.putText(frame, f"Distance: {distance:.2f} m", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Calcul de l'angle d'inclinaison par rapport à la caméra
            rotation_matrix, _ = cv2.Rodrigues(rvecs[0])
            angles_rad = cv2.RQDecomp3x3(rotation_matrix)[0]
            angles_deg = np.degrees(angles_rad)*180/10000
            cv2.putText(frame, f"Angle: {angles_deg[0]:.0f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"{angles_deg[1]:.0f}", (150, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"{angles_deg[2]:.0f} deg", (210, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    #If you want to change the size of the frame
    height, width = frame.shape[:2]
    print(height,width)
    cv2.imshow('Webcam Aruco Detection',cv2.resize(frame, (int(width*0.8),int(height*0.8))))
    #cv2.imshow('Webcam Aruco Detection', frame)

    # Sortir de la boucle si la touche 'q' est pressée
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libérer la capture vidéo et fermer la fenêtre
cap.release()
cv2.destroyAllWindows()
