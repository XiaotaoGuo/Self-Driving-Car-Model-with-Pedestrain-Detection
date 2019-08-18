import os
import time
import numpy as np
from PIL import Image
import glob
from donkeycar.utils import rgb2gray
import cv2

class BaseCamera:

    def run_threaded(self):
        #return self.frame
        return self.frame,self.person

class PiCamera(BaseCamera):
    def __init__(self, image_w=160, image_h=120, image_d=3, framerate=20):
        from picamera.array import PiRGBArray
        from picamera import PiCamera
        
        resolution = (image_w, image_h)
        # initialize the camera and stream
        self.camera = PiCamera() #PiCamera gets resolution (height, width)
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture,
            format="rgb", use_video_port=True)

        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.on = True
        self.image_d = image_d
        self.faceCascade = cv2.CascadeClassifier('/home/pi/opencv-haar-classifier-training/trained_classifiers/indoor_person_cellphone.xml')
        self.person = False
        self.num = 0
        print('PiCamera loaded.. .warming camera')
        time.sleep(2)


    def run(self):
        f = next(self.stream)
        frame = f.array
        self.rawCapture.truncate(0)
        if self.image_d == 1:
            frame = rgb2gray(frame)
        return frame
    
    def detect(self):
        start = time.clock
        img = self.frame
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.faceCascade.detectMultiScale(
            gray,     
            scaleFactor=1.2,
            minNeighbors=5,     
            minSize=(20, 20)
        )
        if len(faces) != 0:
            self.person = True
        else:
            self.person = False
        
        end = time.clock()
        t = end-time
        str = "time:"+str(t)+"s"
        font = cv2.FONT_HERSHEY_SIMPLEX
        img = cv2.putText(img, str, (0, 10), font, 0.5, (255, 255, 255), 1)
        for (x,y,w,h) in faces:
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]  
        cv2.imshow('video',img)
        cv2.waitKey(20)

    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            self.rawCapture.truncate(0)
            #print('Captured %dx%dx%d image' % (self.frame.shape[1], self.frame.shape[0],self.frame.shape[2]))
            self.detect()
            '''
            img = self.frame
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            cv2.imshow('image',img)
            cv2.waitKey(30)
            cv2.imwrite(str(self.num)+'traffic_sign.png',img)
            print(self.num)
            self.num = self.num + 1
            '''
            if self.image_d == 1:
                self.frame = rgb2gray(self.frame)
            # if the thread indicator variable is set, stop the thread
            if not self.on:
                break

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        print('stoping PiCamera')
        time.sleep(.5)
        self.stream.close()
        self.rawCapture.close()
        self.camera.close()

class Webcam(BaseCamera):
    def __init__(self, image_w=160, image_h=120, image_d=3, framerate = 20, iCam = 0):
        import pygame
        import pygame.camera

        super().__init__()
        resolution = (image_w, image_h)
        pygame.init()
        pygame.camera.init()
        l = pygame.camera.list_cameras()
        print('cameras', l)
        self.cam = pygame.camera.Camera(l[iCam], resolution, "RGB")
        self.resolution = resolution
        self.cam.start()
        self.framerate = framerate
        #self.faceCascade = cv2.CascadeClassifier('/home/pi/haarcascades/haarcascade_fullbody.xml')
        #self.person = False

        # initialize variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.on = True
        self.image_d = image_d

        print('WebcamVideoStream loaded.. .warming camera')

        time.sleep(2)
    
    def detect(self):
        img = self.frame
        #img = cv2.flip(img, -1)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.faceCascade.detectMultiScale(
            gray,     
            scaleFactor=1.2,
            minNeighbors=5,     
            minSize=(20, 20)
        )
        if len(faces) != 0:
            print('faces appears!')
            self.person = True
        else:
            self.person = False
        #else:
        #    print('no face')
        
        for (x,y,w,h) in faces:
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]  
        cv2.imshow('video',img)
        cv2.waitKey(20)
    
    def update(self):
        from datetime import datetime, timedelta
        import pygame.image
        while self.on:
            start = datetime.now()

            if self.cam.query_image():
                # snapshot = self.cam.get_image()
                # self.frame = list(pygame.image.tostring(snapshot, "RGB", False))
                snapshot = self.cam.get_image()
                snapshot1 = pygame.transform.scale(snapshot, self.resolution)
                self.frame = pygame.surfarray.pixels3d(pygame.transform.rotate(pygame.transform.flip(snapshot1, True, False), 90))
                if self.image_d == 1:
                    self.frame = rgb2gray(self.frame)

            stop = datetime.now()
            s = 1 / self.framerate - (stop - start).total_seconds()
            if s > 0:
                time.sleep(s)

        self.cam.stop()

    def run_threaded(self):
        return self.person

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        print('stoping Webcam')
        time.sleep(.5)

class MockCamera(BaseCamera):
    '''
    Fake camera. Returns only a single static frame
    '''
    def __init__(self, resolution=(160, 120), image=None):
        if image is not None:
            self.frame = image
        else:
            self.frame = Image.new('RGB', resolution)

    def update(self):
        pass

    def shutdown(self):
        pass

class ImageListCamera(BaseCamera):
    '''
    Use the images from a tub as a fake camera output
    '''
    def __init__(self, path_mask='~/d2/data/**/*.jpg'):
        self.image_filenames = glob.glob(os.path.expanduser(path_mask), recursive=True)
    
        def get_image_index(fnm):
            sl = os.path.basename(fnm).split('_')
            return int(sl[0])

        '''
        I feel like sorting by modified time is almost always
        what you want. but if you tared and moved your data around,
        sometimes it doesn't preserve a nice modified time.
        so, sorting by image index works better, but only with one path.
        '''
        self.image_filenames.sort(key=get_image_index)
        #self.image_filenames.sort(key=os.path.getmtime)
        self.num_images = len(self.image_filenames)
        print('%d images loaded.' % self.num_images)
        print( self.image_filenames[:10])
        self.i_frame = 0
        self.frame = None
        self.update()

    def update(self):
        pass

    def run_threaded(self):        
        if self.num_images > 0:
            self.i_frame = (self.i_frame + 1) % self.num_images
            self.frame = Image.open(self.image_filenames[self.i_frame]) 

        return np.asarray(self.frame)

    def shutdown(self):
        pass
