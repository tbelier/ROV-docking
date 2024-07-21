import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class NicheVision(Node):
    def __init__(self):
        super().__init__('nicheVision')
        self.subscription = self.create_subscription(Image,'/bluerov_ros_camera/image_raw',self.image_callback,10)
        self.imagePublisher = self.create_publisher(Image, '/publishImage', 10)
        self.posePublisher = self.create_publisher(Float64MultiArray, '/vision/pose', 10)
        self.bridge = CvBridge()

        # Paramètres
        self.sizeMarker = 10*1e-2
        self.camera_matrix = np.array([[619.8, 0, 615], [0, 623, 328], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([[-0.10007486,  0.16494157, -0.02190847,  0.00734263, -0.14037809]], dtype=np.float32)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)

        # Matrice de décalage des points mécas : 
        """self.d_meca = np.array([[0.0, 0.0,      0.0, 0.0], #Aruco 1
                                 [0.0, 0.0,     168, 0.0], #Aruco 2
                                 [0.0, -350,   -168, 0.0], #Aruco 3
                                 [0.0, 350, -168, 0.0], #Aruco 
                                 [0.0, -350, 168, 0.0], #Aruco 5
                                 [0.0, 350,168, 0.0], #Aruco 6
                                 [0.0, 0.0, 0.0, 0.0], #Aruco 7
                                 [0.0, 0.0, 0.0, 0.0], #Aruco 8
                                 [0.0, 0.0, 0.0, 0.0]])*1e-3 #Aruco 9"""
        # Matrice de décalage des points mécas : 
        self.d_meca = np.array([[0.0, 0.0,      0.0, 0.0], #Aruco 1
                                 [0.0, 0.0,     -165, 0.0], #Aruco 2
                                 [0.0, -336,   -165, 0.0], #Aruco 3
                                 [0.0, 335, -170, 0.0], #Aruco 
                                 [0.0, -345, 195, 0.0], #Aruco 5
                                 [0.0, 335,180, 0.0], #Aruco 6
                                 [0.0, 0.0, 0.0, 0.0], #Aruco 7
                                 [0.0, 0.0, 0.0, 0.0], #Aruco 8
                                 [0.0, 0.0, 0.0, 0.0]])*1e-3 #Aruco 9
        
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        detected_image, Lposes = self.detect(frame, self.sizeMarker)
        msgImage = self.bridge.cv2_to_imgmsg(detected_image, 'bgr8')
        self.imagePublisher.publish(msgImage)

        msgPose = Float64MultiArray()
        computedLpose = self.computePose(Lposes)
        #print("msgPose.data :" + str(msgPose.data))
        for k in range(len(computedLpose)):
            #print("Lpose[k] : "+ str(computedLpose[k]))
            msgPose.data.append(computedLpose[k])
        self.posePublisher.publish(msgPose)

    def image_treatment(self, frame):
        clahe = cv2.createCLAHE(clipLimit=3., tileGridSize=(10, 10))
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)  # convert from BGR to LAB color space
        l, a, b = cv2.split(lab)  # split on 3 different channels
        l2 = clahe.apply(l)  # apply CLAHE to the L-channel
        lab = cv2.merge((l2, a, b))  # merge channels
        cv_image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)  # convert from LAB to BGR

        gamma = 3#0.7  ##corrige la brightness non linéairement
        lookUpTable = np.array([np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255) for i in range(256)], dtype=np.uint8)
        return cv2.LUT(cv_image, lookUpTable)
    
    def detect(self, frame, sizeMarker, resize_coeff=1):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict)
        L = []
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i, marker_id in enumerate(ids):
                # COMPUTING
                print("i, marker_id", i, marker_id)
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], sizeMarker, self.camera_matrix, self.dist_coeffs)
                #print("tvecs",tvecs)
                #print("rvecs",rvecs)
                # DISTANCE
                distance = np.linalg.norm(tvecs[0])
                
                

                #ORIENTATION
                rotation_matrix, _ = cv2.Rodrigues(rvecs[0])
                angles_rad = cv2.RQDecomp3x3(rotation_matrix)[0]
                angles_deg = np.degrees(angles_rad)*180/10000
                
                
                dy,dz,dx = -tvecs[0][0][0],-tvecs[0][0][1],tvecs[0][0][2] #ATTENTION A L'ORDRE ET AU SIGNE !!
                dcap = angles_deg[1]
                L.append([marker_id[0], dx, dy, dz, dcap])

                cv2.putText(frame, f"Distance: {distance:.2f} m", (10, 30),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(frame, f"Angle: {angles_deg[0]:.0f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(frame, f"{angles_deg[1]:.0f}", (150, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(frame, f"{angles_deg[2]:.0f} deg", (210, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        #print(L)

        return frame, L
    
    def computePose(self, Lpose):
        nbDetected = len(Lpose)
        if nbDetected == 0 : return []
        else :
            dx_pose, dy_pose, dz_pose, dcap_pose =  0.0, 0.0, 0.0, 0.0 #initialisation
            #print("--------- : " +str(Lpose))
            for k in range(nbDetected):
                markerId_k, dx_k,dy_k,dz_k,dcap_k = Lpose[k]
                if 1 <= markerId_k <= 8 : # TOOOOOOOOOOOOOOOOOOOOOOOOODDDDDDDDDDDDDDDDDDDDDOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO : changer à 9
                    #print("------------ markerId : " + str(markerId_k))
                    dx_meca, dy_meca, dz_meca, dcap_meca = self.d_meca[markerId_k-1] 


                    dx_pose   += dx_k+dx_meca #attention j'additionne les di, le signe lié à la position de l'aruco se cache dans ls listes di_meca
                    dy_pose   += dy_k+dy_meca
                    dz_pose   += dz_k+dz_meca
                    dcap_pose += dcap_k+dcap_meca
                    self.get_logger().info("-----------")
                    self.get_logger().info("marker " + str(markerId_k-1) + " : " +str(self.d_meca[markerId_k-1]))
                    self.get_logger().info("dx_pose : "+str(dx_pose))
                    self.get_logger().info("dx_k : "+str(dx_k))
                    self.get_logger().info("dx_meca : "+str(dx_meca))

                    self.get_logger().info("dy_pose : "+str(dy_pose))
                    self.get_logger().info("dy_k : "+str(dy_k))
                    self.get_logger().info("dy_meca : "+str(dy_meca))

                    self.get_logger().info("dz_pose : "+str(dz_pose))
                    self.get_logger().info("dz_k : "+str(dz_k))
                    self.get_logger().info("dz_meca : "+str(dz_meca))

                    self.get_logger().info("dcap_pose : "+str(dcap_pose))
                    self.get_logger().info("dcap_k : "+str(dcap_k))
                    self.get_logger().info("dcap_meca : "+str(dcap_meca))

                else :
                    self.get_logger().info("\033[91ATTENTION : LE ROV A DETECTE L'ARUCO :  + str(markerId_k) NOT ANALYZED\33\0m")

            return [nbDetected, dx_pose/nbDetected, dy_pose/nbDetected, dz_pose/nbDetected, dcap_pose/nbDetected] #on renvoie le nombre d'Aruco détectés et la pose
    

def main(args=None):
    rclpy.init(args=args)
    nicheVision = NicheVision()
    rclpy.spin(nicheVision)
    nicheVision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()