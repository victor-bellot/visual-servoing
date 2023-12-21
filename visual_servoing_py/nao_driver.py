import cv2
from naoqi import ALProxy
import numpy as np
import os
import time
import signal

# import Image
try:
    from PIL import Image
except:
    import Image
import sys


# doc is made in html format with pdoc
# pip install pdoc
# pdoc --html --html-dir ../html-doc --overwrite  nao_driver.py 
# cp ../html-doc/nao_driver.m.html /mnt/webperso/zerrbe/filerepo/vsik/

class NaoDriver:
    """ NaoDriver : gives access to real or virtual NAO """

    def __init__(self, nao_ip="localhost", nao_port=11212):
        """ Creates NAO driver to handle virtual and real robots.   
        nao_ip : string with the IP address of the robot. Default "localhost" for virtual NAO on V-REP.
        nao_port : integer with the acces port to the robot. Default 11212 for virtual NAO on V-REP.
        """
        __docformat__ = "restructuredtext"
        self.__nao_port = nao_port
        self.__nao_ip = nao_ip
        self.vnao = True
        """ True if virtual NAO, False if real NAO """
        self.__vnao_path = "/home/victor/nao/UE52-VS-IK/imgs"
        self.__vnao_frame = 0
        self.__vnao_image = "out_%5.5d.ppm" % (self.__nao_port)
        self.__vnao_camera_image = os.path.join(self.__vnao_path, self.__vnao_image)
        self.__cam_num = 0
        # initialize NAO
        self.posture_proxy = None
        """ Posture proxy to send posture commands to NAO
        You can use all the functions listed here :
        http://doc.aldebaran.com/2-1/naoqi/motion/alrobotposture-api.html#alrobotposture-api
        """
        self.motion_proxy = None
        """ Motion Proxy to send motion commands to NAO
        You can use all the functions listed here :
        http://doc.aldebaran.com/2-1/naoqi/motion/almotion-api.html
        """
        self.__init_motion()
        self.__init_posture()
        self.__video_client = None
        self.image_width = 0
        """ Image horizontal size in pixels """
        self.image_height = 0
        """ Image vertical size in pixels """
        self.image = None
        self.camera_proxy = None
        """ Camera Proxy to access real NAO's video 
        In principle, you do not need to use it, as function to retrieve images are already given
        But you can use all the functions listed here :
        http://doc.aldebaran.com/2-1/naoqi/vision/alvideodevice-api.html#alvideodevice-api
        """
        self.__camera_resolution = None
        self.__camera_color_space = None
        self.__camera_fps = None
        self.__init_camera(self.__cam_num)
        # trap ctrl-C to cleanly stop the robot and put it at rest
        signal.signal(signal.SIGINT, self.__clean_kill_nao)

    def __clean_kill_nao(self, signal, frame):
        print "pgm interrupted, put NAO is safe pose ..."
        self.motion_proxy.stopMove()
        self.posture_proxy.goToPosture("Crouch", 0.5)
        time.sleep(0.5)
        stiffnesses = 0.0
        self.motion_proxy.setStiffnesses(["Body"], stiffnesses)
        exit()

    def __check_constant_green_image(self, img, width, height):
        tstpix = []
        tstpix.append(img[0, 0])
        tstpix.append(img[0, width - 1])
        tstpix.append(img[height - 1, 0])
        tstpix.append(img[height - 1, width - 1])
        cstgreen = True
        for pix in tstpix:
            if pix[0] != 0:
                cstgreen = False
                break
            if (pix[1] != 154) and (pix[1] != 135):
                cstgreen = False
                break
            if pix[2] != 0:
                cstgreen = False
                break
        return cstgreen

    def __init_motion(self):
        print("ip", self.__nao_ip, "port", self.__nao_port)
        try:
            self.motion_proxy = ALProxy("ALMotion", self.__nao_ip, self.__nao_port)
        except Exception, e:
            print "Could not create proxy to ALMotion"
            print("Error was: ", e)
            exit(1)

    def __init_posture(self):
        try:
            self.posture_proxy = ALProxy("ALRobotPosture", self.__nao_ip, self.__nao_port)
        except Exception, e:
            print "Could not create proxy to ALPosture"
            print("Error was: ", e)
            exit(1)

    def set_nao_at_rest(self):
        """ Set NAO in a safe (rest) position : no current in servos, straight posture """
        stiffnesses = 1.0
        self.motion_proxy.wakeUp()
        self.posture_proxy.goToPosture("Straight", 0.5)
        # relax all servos by removing current (prevent over heating)
        stiffnesses = 0.0
        self.motion_proxy.setStiffnesses(["Body"], stiffnesses)
        self.motion_proxy.rest()

    def __set_camera_subscriber(self):
        try:
            lSubs = self.camera_proxy.getSubscribers()
            for subs in lSubs:
                if subs.startswith("python_client"):
                    self.camera_proxy.unsubscribe(subs)
        except:
            print "cannot unsubscribe"
            pass
        try:
            self.__video_client = self.camera_proxy.subscribeCamera("python_client", self.__cam_num,
                                                                    self.__camera_resolution, self.__camera_color_space,
                                                                    self.__camera_fps)
        except:
            print "pb with subscribe"
            lSubs = self.camera_proxy.getSubscribers()
            for subs in lSubs:
                if subs.startswith("python_client"):
                    self.camera_proxy.unsubscribe(subs)
            self.__video_client = self.camera_proxy.subscribeCamera("python_client", self.__cam_num,
                                                                    self.__camera_resolution, self.__camera_color_space,
                                                                    self.__camera_fps)
        print self.camera_proxy.getSubscribers()
        print("videoClient ", self.__video_client)

    def __init_camera(self, cam_num):
        self.__cam_num = cam_num  # 0:top cam, 1: bottom cam
        self.camera_proxy = ALProxy("ALVideoDevice", self.__nao_ip, self.__nao_port)
        self.__camera_resolution = 1  # 0 : QQVGA, 1 : QVGA, 2 : VGA
        self.__camera_color_space = 11  # RGB
        self.__camera_fps = 4  # frames Per Second
        self.camera_proxy.setActiveCamera(self.__cam_num)
        print("Active camera is", self.camera_proxy.getActiveCamera())
        self.__set_camera_subscriber()
        img_ok, img, nx, ny = self.__get_real_image()
        # if image is constant green, then we are on the simulator (no actual video frame)
        cstGreen = self.__check_constant_green_image(img, nx, ny)
        if cstGreen:
            print "run on simulated NAO, no video frame, use still images"
            self.vnao = True
        else:
            print "run on real NAO"
            self.vnao = False

    def set_virtual_camera_path(self, path):
        """
        Change the path to access images from the V-REP NAO simulator. 
        Input : 
           path (string) : absolute path to NAO camera files produced by V-REP simulator
        """
        self.__vnao_path = path
        self.__vnao_frame = 0
        self.__vnao_camera_image = os.path.join(self.__vnao_path, self.__vnao_image)
        print("virtual nao image", self.__vnao_camera_image)

    def get_cam_num(self):
        """ return the current camera. 0: Top , 1: Bottom"""
        return self.__cam_num

    def change_camera (self, cam_num):
        """ Change NAO's active camera. cam_num = 0 : top camera, cam_num = 1 : bottom camera """
        self.__cam_num = cam_num
        if self.vnao:
            if self.__cam_num == 0:
                self.__vnao_image = "out_%5.5d.ppm"%(self.__nao_port)
            else:
                self.__vnao_image = "out_down_%5.5d.ppm"%(self.__nao_port)
            self.__vnao_camera_image=os.path.join(self.__vnao_path,self.__vnao_image)
            print ("virtual nao image",self.__vnao_camera_image)    
            print "Active camera is",self.__cam_num
        else:   
            self.camera_proxy.setActiveCamera(self.__cam_num)
            self.__set_camera_subscriber()
            print "Active camera is",self.camera_proxy.getActiveCamera()

    def __get_real_image(self):
        # Get a camera image.
        # image[6] contains the image data passed as an array of ASCII chars.
        nao_image = self.camera_proxy.getImageRemote(self.__video_client)
        image_width = nao_image[0]
        image_height = nao_image[1]
        image_array = nao_image[6]
        # Create a PIL Image from our pixel array.
        # pilImg = Image.fromstring("RGB", (imageWidth, imageHeight), array)
        pil_img = Image.frombytes("RGB", (image_width, image_height), image_array)
        # Convert Image to OpenCV
        cv_img = np.array(pil_img)
        # Convert RGB to BGR 
        cv_img = cv_img[:, :, ::-1].copy()
        self.image_width = image_width
        self.image_height = image_height
        self.image = cv_img
        img_ok = True
        return img_ok, self.image, self.image_width, self.image_height

    def __get_vrep_image(self):
        imgok = False
        # print("__get_vrep_image: virtual nao image",self.__vnao_camera_image)
        while not imgok:
            try:
                cv_img = cv2.imread(self.__vnao_camera_image)
                image_height, image_width, image_channels = cv_img.shape
                imgok = True
                self.image_width = image_width
                self.image_height = image_height
                # flip image vertically
                self.image = cv2.flip(cv_img, 0)
            except Exception, e:
                print("Can't read image %s, retry ..." % (self.__vnao_camera_image))
                imgok = False
                time.sleep(0.05)

        return imgok, self.image, self.image_width, self.image_height

    def get_image(self):
        """ get the current image in the stream. 
        Input :
 
        Output : 
          imgOk (boolean) : true if image is valid
          image (array) : image in open cv format
          imageWidth (integer) : horizontal size
          imageHeight (integer) : vertical size
        """
        if self.vnao:
            img_ok, cv_img, image_width, image_height = self.__get_vrep_image()
        else:
            img_ok, cv_img, image_width, image_height = self.__get_real_image()
        return img_ok, cv_img, image_width, image_height

    def show_image(self, key=1):
        """ show the last acquired image in a window
        Input : 
          key (integer) : time to wait in ms

        if key is 0, wait for a click
        if key is n (non zero), wait for n ms
        default is 1 ms 
        """
        # define display window
        window_name = "NAO Camera"
        cv2.namedWindow(window_name)
        cv2.resizeWindow(window_name, self.image_width, self.image_height)
        cv2.moveWindow(window_name, 0, 0)
        cv2.imshow(window_name, self.image)
        cv2.waitKey(key)


if __name__ == "__main__":
    # virtual NAO on current computer : nao_ip="localhost", nao_port=11212
    # virtual NAO on other computer : nao_ip="172.20.22.34", nao_port=11212
    # real NAO Blue : nao_ip="172.20.25.151", nao_port=9559

    cv2.setNumThreads(0)
    cv2.ocl.setUseOpenCL(False)
    # default virtual NAO on V-REP
    #nao_ip = "localhost"
    nao_ip = "172.17.0.1"
    nao_port = 11212

    # change IP and port with arguments in the command line
    try:
        nao_ip = sys.argv[1]
    except:
        pass
    try:
        nao_port = int(sys.argv[2])
    except:
        pass

    # create NAO driver
    nao_drv = NaoDriver(nao_ip=nao_ip, nao_port=nao_port)

    # put NAO in safe position
    nao_drv.set_nao_at_rest()

    # Important  when using virtual NAO !!! set path to the folder where V-REP stores the camera images
    # nao_drv.set_virtual_camera_path("/home/victor/nao/UE52-VS-IK/imgs")
    nao_drv.set_virtual_camera_path("/home/dockeruser/shared/imgs")

    fps = 4
    dt_loop = 1. / fps
    # infinite test loop, stops with Ctrl-C
    while True:
        t0_loop = time.time()
        img_ok, img, nx, ny = nao_drv.get_image()
        nao_drv.show_image(key=1)
        dt = dt_loop - (time.time() - t0_loop)
        if dt > 0:
            time.sleep(dt)
