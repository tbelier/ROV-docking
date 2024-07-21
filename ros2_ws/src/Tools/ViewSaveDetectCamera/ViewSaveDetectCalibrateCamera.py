# !/usr/bin/env python3
# coding=utf-8

"""
BlueRov video capture class
"""

from PIL import Image, ImageFont, ImageDraw

import cv2
import gi
import numpy as np
import cv2.aruco as aruco
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import time
import glob
import pickle


class Video():
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
    """

    def __init__(self, port=5600):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self._frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.resolution = (640, 480)  # ajout de ma part

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()

        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)

        resolution = (
            caps.get_structure(0).get_value('width'),
            caps.get_structure(0).get_value('height')

        )

        return array, resolution

    def frame(self):
        """ Get Frame

        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """Check if frame is available

        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)

    def run(self):
        """ Get frame to update _frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf,

            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame, new_resolution = self.gst_to_opencv(sample)
        self._frame = new_frame

        self.resolution = new_resolution

        return Gst.FlowReturn.OK

def display(video):

    while True:
        # Wait for the next frame
        if not video.frame_available():
            continue

        frame = video.frame()

        # ajout des informations
        font = cv2.FONT_HERSHEY_SIMPLEX
        text1 = "TEST"
        text2 = "TEST2"
        cv2.putText(frame, text1, (10, 20), font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, text2, (10, 40), font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


def save_video(title_video, video):
    fps = 20  # NE PAS METTRE < 10fps !!
    frame_width = int(video.resolution[0])
    frame_height = int(video.resolution[1])
    size = (frame_width, frame_height)
    result = cv2.VideoWriter(title_video,cv2.VideoWriter_fourcc(*'XVID'),fps, size)
    print('resolution video : ', size)
    while True:
        # Wait for the next frame
        if not video.frame_available():
            continue

        frame = video.frame()
        result.write(frame)
        time.sleep(1 / fps)

        if time.time() > t0 + 100:
            break

        # Affichage
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break




def getImages(video):
    num = 0

    while True:

        ret, frame = video.frame_available(), video._frame

        k = cv2.waitKey(5)

        if k == 27:
            break
        elif k == ord('s'):  # wait for 's' key to save and exit
<<<<<<< HEAD
            cv2.imwrite('imagesCalibration/img' + str(num) + '.png', frame)
=======
            cv2.imwrite('/home/titouan/niche/ros2_ws/src/Tools/ViewSaveDetectCamera/imagesCalibration/img' + str(num) + '.png', frame)
>>>>>>> develop
            print("image saved!")
            num += 1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        cv2.imshow('Img', frame)

def calibration():

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2)

    #size_of_chessboard_squares_mm = 1000 #TODO : change this value
    #objp = objp * size_of_chessboard_squares_mm

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

<<<<<<< HEAD
    images = glob.glob('imagesCalibration/*.png')
=======
    images = glob.glob('ros2_ws/src/Tools/ViewSaveDetectCamera/imagesCalibration/*.png')
    print(len(images))
>>>>>>> develop

    for image in images:
        img = cv2.imread(image)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, chessboardSize, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(0)

    cv2.destroyAllWindows()

    ############## CALIBRATION #######################################################

    ret, cameraMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frameSize, None, None)
    print("cameraMatrix : " + str(cameraMatrix))
    print("dist : " + str(dist))
    # Save the camera calibration result for later use (we won't worry about rvecs / tvecs)
    pickle.dump((cameraMatrix, dist), open("calibration.pkl", "wb"))
    pickle.dump(cameraMatrix, open("cameraMatrix.pkl", "wb"))
    pickle.dump(dist, open("dist.pkl", "wb"))

def detection(video):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)

    while True:
        ret, frame = video.frame_available(), video._frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i, marker_id in enumerate(ids):
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], markerLength, camera_matrix, dist_coeffs)
                distance = filter_median(L_distance, np.linalg.norm(tvecs[0]))
                cv2.putText(frame, f"Distance: {distance:.2f} m", (10, 30),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                rotation_matrix, _ = cv2.Rodrigues(rvecs[0])
                angles_rad = cv2.RQDecomp3x3(rotation_matrix)[0]
                angles_deg = np.degrees(angles_rad) * 180 / 10000
                angles_deg[0] = filter_median(L_angle0, angles_deg[0])

                angles_deg[1] = filter_median(L_angle1, angles_deg[1])
                angles_deg[2] = filter_median(L_angle2, angles_deg[2])
                cv2.putText(frame, f"Angle: {angles_deg[0]:.0f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(frame, f"{angles_deg[1]:.0f}", (150, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(frame, f"{angles_deg[2]:.0f} deg", (210, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.imshow('Webcam Aruco Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def save_detection(title_video, video):
    #Initialize detection
    print("Initialize detection...")

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)

    #Initialize saving
    print("Initialize saving...")
    fps = 20  # NE PAS METTRE < 10fps !!
    frame_width = int(video.resolution[0])
    frame_height = int(video.resolution[1])
    size = (frame_width, frame_height)
    result = cv2.VideoWriter(title_video,cv2.VideoWriter_fourcc(*'XVID'),fps, size)




    while True:
        ret, frame = video.frame_available(), video._frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


        # Détection des marqueurs Aruco
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)

        # Dessiner les marqueurs détectés sur l'image
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            # Affichage des IDs des marqueurs détectés
            for i, marker_id in enumerate(ids):
                # Calcul de la pose (position et orientation) du marqueur
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], markerLength, camera_matrix, dist_coeffs)

                # Calcul de la distance entre la caméra et le marqueur
                distance = filter_median(L_distance, np.linalg.norm(tvecs[0]))
                cv2.putText(frame, f"Distance: {distance:.2f} m", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Calcul de l'angle d'inclinaison par rapport à la caméra
                rotation_matrix, _ = cv2.Rodrigues(rvecs[0])
                angles_rad = cv2.RQDecomp3x3(rotation_matrix)[0]
                angles_deg = np.degrees(angles_rad) * 180 / 10000
                cv2.putText(frame, f"Angle: {angles_deg[0]:.0f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                cv2.putText(frame, f"{angles_deg[1]:.0f}", (150, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                cv2.putText(frame, f"{angles_deg[2]:.0f} deg", (210, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


        result.write(frame)

        time.sleep(1 / fps)
        if time.time() > t0 + 100: #TODO : changer 100 pour ajouter la durée
            break

        # Affichage de la vidéo avec les marqueurs détectés
        cv2.imshow('Webcam Aruco Detection', frame)

        # Sortir de la boucle si la touche 'q' est pressée
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def filter_median(values, new_value):
    values.append(new_value)  # Ajouter la nouvelle valeur à la liste

    if len(values) < 10:
        return values[0]

    else:

        average =np.median(np.array(values))
        values.pop(0)

        return average



if __name__ == '__main__':

    Affichage = 0
    Enregistrement = 0
    Detection = 0
    Enregistrement_Detection = 0
    GetImages = 0
    Calibration = 1

    markerLength = 0.164
    camera_matrix = np.array([[702, 0, 617], [0, 698, 393], [0, 0, 1]], dtype=np.float32)  # Matrice de la caméra
    #camera_matrix = np.array([[755, 0, 634], [0, 750, 360], [0, 0, 1]], dtype=np.float32)  # Matrice de la caméra
    dist_coeffs = np.array([[ 0.06991307, -0.10688273, -0.00414977,  0.01193672,  0.09424899]], dtype=np.float32)
<<<<<<< HEAD
    chessboardSize = (8, 5)  # TODO : change this value0
=======
    chessboardSize = (9, 6)  # TODO : change this value0
>>>>>>> develop
    frameSize = (1280, 720)  # TODO : change this value

    L_distance = []
    L_angle0 = []
    L_angle1 = []
    L_angle2 = []

    if Calibration == 1:
        print("Calibration ...")
        calibration()
    else :
        print('connexion camera...')
        video = Video(5600)
        test = 0
        while test == 0:
            # Wait for the next frame
            if not video.frame_available():
                print("Initializing the video.")
                continue

            frame = video.frame()  # pour mettre à jour les parametres comme la résolution
            test = 1


    #### Afficher la camera
    if Affichage == 1:
        display(video)

    ### Enregistrer une video
    t0 = time.time()

    if Enregistrement == 1:
        print("Saving the video without detection ...")
        title_video = 'test_without_detection.avi'
        save_video(title_video, video)

    if Detection == 1:
        detection(video)

    if Enregistrement_Detection == 1:
        print("Saving the detection video ...")
        title_video = 'test_detection.avi'
        save_detection(title_video, video)

    if GetImages == 1:
        print("Getting Images ...")
        print("Press 'S' to saves images ...")
        getImages(video)


