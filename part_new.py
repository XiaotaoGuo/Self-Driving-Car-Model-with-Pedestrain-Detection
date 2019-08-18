import numpy as np
import cv2
import time

class detector():

    def __init__(self):
        self.img_arr = None
        self.throttle = 0.0
        self.person = False
        self.on = True
        self.faceCascade = cv2.CascadeClassifier('/home/pi/haarcascades/haarcascade_frontalface_default.xml')

    def run_threaded(self,img):
        self.img_arr = img
        #self.detect_frontface()
        #print('hi')
        return self.throttle, self.person

    def run(self, img=None):
        raise Exception("We expect for this part to be run with the threaded=True argument.")
        return None, None, None, None

    def update(self):
        #detect_frontface(self)
        print('hihihihi')

    def detect_frontface(self):
        img = self.img_arr
        img = cv2.flip(img, -1)
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
        #else:
        #    print('no face')
        
        for (x,y,w,h) in faces:
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]  
        cv2.imshow('video',img)
        cv2.waitKey(20)
        
        return True

    
    def shutdown(self):
        pass

class drive_mode_select():

    def __init__(self):
        self.throttle = 0.0
        self.angle = 0.0
        self.person = False
        self.t = 0
        
    def run(self,pilot_throttle,pilot_angle,person):
        self.throttle = pilot_throttle
        self.angle = pilot_angle
            
        if self.person == False:
            if person:
                print('see person!')
                self.throttle = 0.0
                self.person = True
                self.t = 0
        else:
            self.throttle = 0.0
            self.t = self.t + 1
            if self.t == 20:
                self.person = False
                
        
        '''
        if self.start_from_stop == False:
            #car doesn't start from stop sign
            if self.run_or_not == True: #car is running or not
                if stop:
                    self.throttle = 0.0
                    self.run_or_not = False
            else:
                time.sleep(3)
                print('Stop:3s,Go!')
                self.throttle = 0.3
                self.t = 0
                self.start_from_stop = True
        else:
            self.t = self.t + 1
            if self.t == 20:
                self.start_from_stop = False
        '''
        
        self.angle = pilot_angle
        print('throttle:',self.throttle)

        return self.angle,self.throttle
    
    def shutdown(self):
        pass
