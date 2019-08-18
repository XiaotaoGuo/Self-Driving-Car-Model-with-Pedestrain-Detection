'''

pilots.py

Methods to create, use, save and load pilots. Pilots 
contain the highlevel logic used to determine the angle
and throttle of a vehicle. Pilots can include one or more 
models to help direct the vehicles motion. 

'''




import os
import numpy as np
import keras

import donkeycar as dk


class KerasPilot(object):
    def __init__(self):
        self.model = None
        self.optimizer = "adam"
 
    def load(self, model_path):
        self.model = keras.models.load_model(model_path)

    def load_weights(self, model_path, by_name=True):
        self.model.load_weights(model_path, by_name=by_name)

    def shutdown(self):
        pass

    def compile(self):
        pass

    def set_optimizer(self, optimizer_type, rate, decay):
        if optimizer_type == "adam":
            self.model.optimizer = keras.optimizers.Adam(lr=rate, decay=decay)
        elif optimizer_type == "sgd":
            self.model.optimizer = keras.optimizers.SGD(lr=rate, decay=decay)
        elif optimizer_type == "rmsprop":
            self.model.optimizer = keras.optimizers.RMSprop(lr=rate, decay=decay)
        else:
            raise Exception("unknown optimizer type: %s" % optimizer_type)
    
    def train(self, train_gen, val_gen, 
              saved_model_path, epochs=100, steps=100, train_split=0.8,
              verbose=1, min_delta=.0005, patience=5, use_early_stop=True):
        
        """
        train_gen: generator that yields an array of images an array of 
        
        """

        #checkpoint to save model after each epoch
        save_best = keras.callbacks.ModelCheckpoint(saved_model_path, 
                                                    monitor='val_loss', 
                                                    verbose=verbose, 
                                                    save_best_only=True, 
                                                    mode='min')
        
        #stop training if the validation error stops improving.
        early_stop = keras.callbacks.EarlyStopping(monitor='val_loss', 
                                                   min_delta=min_delta, 
                                                   patience=patience, 
                                                   verbose=verbose, 
                                                   mode='auto')
        
        callbacks_list = [save_best]

        if use_early_stop:
            callbacks_list.append(early_stop)
        
        hist = self.model.fit_generator(
                        train_gen, 
                        steps_per_epoch=steps, 
                        epochs=epochs, 
                        verbose=1, 
                        validation_data=val_gen,
                        callbacks=callbacks_list, 
                        validation_steps=steps*(1.0 - train_split))
        return hist


class KerasCategorical(KerasPilot):
    def __init__(self, input_shape=(120, 160, 3), *args, **kwargs):
        super(KerasCategorical, self).__init__(*args, **kwargs)
        self.model = default_categorical(input_shape)
        self.compile()

    def compile(self):
        self.model.compile(optimizer=self.optimizer, metrics=['acc'],
                  loss={'angle_out': 'categorical_crossentropy', 
                        'throttle_out': 'categorical_crossentropy'},
                  loss_weights={'angle_out': 0.5, 'throttle_out': 1.0})
        
    def run(self, img_arr):
        if img_arr is None:
            print('no image')
            return 0.0, 0.0

        img_arr = img_arr.reshape((1,) + img_arr.shape)
        angle_binned, throttle = self.model.predict(img_arr)
        #in order to support older models with linear throttle,
        #we will test for shape of throttle to see if it's the newer
        #binned version.
        N = len(throttle[0])
        
        if N > 0:
            throttle = dk.utils.linear_unbin(throttle, N=N, offset=0.0, R=0.5)
        else:
            throttle = throttle[0][0]
        angle_unbinned = dk.utils.linear_unbin(angle_binned)
        return angle_unbinned, throttle
    
    
    
class KerasLinear(KerasPilot):
    def __init__(self, num_outputs=2, input_shape=(120, 160, 3), *args, **kwargs):
        super(KerasLinear, self).__init__(*args, **kwargs)
        self.model = default_n_linear(num_outputs, input_shape)
        self.compile()

    def compile(self):
        self.model.compile(optimizer=self.optimizer,
                loss='mse')

    def run(self, img_arr):
        img_arr = img_arr.reshape((1,) + img_arr.shape)
        outputs = self.model.predict(img_arr)
        steering = outputs[0]
        throttle = outputs[1]
        return steering[0][0], throttle[0][0]



class KerasIMU(KerasPilot):
    '''
    A Keras part that take an image and IMU vector as input,
    outputs steering and throttle

    Note: When training, you will need to vectorize the input from the IMU.
    Depending on the names you use for imu records, something like this will work:

    X_keys = ['cam/image_array','imu_array']
    y_keys = ['user/angle', 'user/throttle']
    
    def rt(rec):
        rec['imu_array'] = np.array([ rec['imu/acl_x'], rec['imu/acl_y'], rec['imu/acl_z'],
            rec['imu/gyr_x'], rec['imu/gyr_y'], rec['imu/gyr_z'] ])
        return rec

    kl = KerasIMU()

    tubgroup = TubGroup(tub_names)
    train_gen, val_gen = tubgroup.get_train_val_gen(X_keys, y_keys, record_transform=rt,
                                                    batch_size=cfg.BATCH_SIZE,
                                                    train_frac=cfg.TRAIN_TEST_SPLIT)

    '''
    def __init__(self, model=None, num_outputs=2, num_imu_inputs=6, input_shape=(120, 160, 3), *args, **kwargs):
        super(KerasIMU, self).__init__(*args, **kwargs)
        self.num_imu_inputs = num_imu_inputs
        self.model = default_imu(num_outputs = num_outputs, num_imu_inputs = num_imu_inputs, input_shape=input_shape)
        self.compile()

    def compile(self):
        self.model.compile(optimizer=self.optimizer,
                  loss='mse')
        
    def run(self, img_arr, accel_x, accel_y, accel_z, gyr_x, gyr_y, gyr_z):
        #TODO: would be nice to take a vector input array.
        img_arr = img_arr.reshape((1,) + img_arr.shape)
        imu_arr = np.array([accel_x, accel_y, accel_z, gyr_x, gyr_y, gyr_z]).reshape(1,self.num_imu_inputs)
        outputs = self.model.predict([img_arr, imu_arr])
        steering = outputs[0]
        throttle = outputs[1]
        return steering[0][0], throttle[0][0]


class KerasBehavioral(KerasPilot):
    '''
    A Keras part that take an image and Behavior vector as input,
    outputs steering and throttle
    '''
    def __init__(self, model=None, num_outputs=2, num_behavior_inputs=2, input_shape=(120, 160, 3), *args, **kwargs):
        super(KerasBehavioral, self).__init__(*args, **kwargs)
        self.model = default_bhv(num_outputs = num_outputs, num_bvh_inputs = num_behavior_inputs, input_shape=input_shape)
        self.compile()

    def compile(self):
        self.model.compile(optimizer=self.optimizer,
                  loss='mse')
        
    def run(self, img_arr, state_array):        
        img_arr = img_arr.reshape((1,) + img_arr.shape)
        bhv_arr = np.array(state_array).reshape(1,len(state_array))
        angle_binned, throttle = self.model.predict([img_arr, bhv_arr])
        #in order to support older models with linear throttle,
        #we will test for shape of throttle to see if it's the newer
        #binned version.
        N = len(throttle[0])
        
        if N > 0:
            throttle = dk.utils.linear_unbin(throttle, N=N, offset=0.0, R=0.5)
        else:
            throttle = throttle[0][0]
        angle_unbinned = dk.utils.linear_unbin(angle_binned)
        return angle_unbinned, throttle


class KerasLocalizer(KerasPilot):
    '''
    A Keras part that take an image and Behavior vector as input,
    outputs steering and throttle, and localisation category
    '''
    def __init__(self, model=None, num_outputs=2, num_behavior_inputs=2, num_locations=8, input_shape=(120, 160, 3), *args, **kwargs):
        super(KerasLocalizer, self).__init__(*args, **kwargs)
        self.model = default_loc(num_outputs = num_outputs, num_locations=num_locations, input_shape=input_shape)
        self.compile()

    def compile(self):
        self.model.compile(optimizer=self.optimizer, metrics=['acc'],
                  loss='mse')
        
    def run(self, img_arr):        
        img_arr = img_arr.reshape((1,) + img_arr.shape)
        angle_binned, throttle, track_loc = self.model.predict([img_arr])
        #in order to support older models with linear throttle,
        #we will test for shape of throttle to see if it's the newer
        #binned version.
        N = len(throttle[0])
        print("track_loc", np.argmax(track_loc[0]), track_loc, track_loc.shape)
        #print("lane", np.argmax(lane[0]), lane, lane.shape)
        
        if N > 0:
            throttle = dk.utils.linear_unbin(throttle, N=N, offset=0.0, R=0.5)
        else:
            throttle = throttle[0][0]
        angle_unbinned = dk.utils.linear_unbin(angle_binned)
        print("angle_unbinned", angle_unbinned, "throttle", throttle)
        
        return angle_unbinned, throttle

def default_categorical(input_shape=(120, 160, 3)):
    from keras.layers import Input, Dense
    from keras.models import Model
    from keras.layers import Convolution2D, MaxPooling2D, Reshape, BatchNormalization
    from keras.layers import Activation, Dropout, Flatten, Dense    
    
    opt = keras.optimizers.Adam()
    drop = 0.2

    img_in = Input(shape=input_shape, name='img_in')                      # First layer, input layer, Shape comes from camera.py resolution, RGB
    x = img_in
    x = Convolution2D(24, (5,5), strides=(2,2), activation='relu', name="conv2d_1")(x)       # 24 features, 5 pixel x 5 pixel kernel (convolution, feauture) window, 2wx2h stride, relu activation
    x = Dropout(drop)(x)                                                      # Randomly drop out (turn off) 10% of the neurons (Prevent overfitting)
    x = Convolution2D(32, (5,5), strides=(2,2), activation='relu', name="conv2d_2")(x)       # 32 features, 5px5p kernel window, 2wx2h stride, relu activatiion
    x = Dropout(drop)(x)                                                      # Randomly drop out (turn off) 10% of the neurons (Prevent overfitting)
    if input_shape[0] > 32 :
        x = Convolution2D(64, (5,5), strides=(2,2), activation='relu', name="conv2d_3")(x)       # 64 features, 5px5p kernal window, 2wx2h stride, relu
    else:
        x = Convolution2D(64, (3,3), strides=(1,1), activation='relu', name="conv2d_3")(x)       # 64 features, 5px5p kernal window, 2wx2h stride, relu
    if input_shape[0] > 64 :
        x = Convolution2D(64, (3,3), strides=(2,2), activation='relu', name="conv2d_4")(x)       # 64 features, 3px3p kernal window, 2wx2h stride, relu
    elif input_shape[0] > 32 :
        x = Convolution2D(64, (3,3), strides=(1,1), activation='relu', name="conv2d_4")(x)       # 64 features, 3px3p kernal window, 2wx2h stride, relu
    x = Dropout(drop)(x)                                                      # Randomly drop out (turn off) 10% of the neurons (Prevent overfitting)
    x = Convolution2D(64, (3,3), strides=(1,1), activation='relu', name="conv2d_5")(x)       # 64 features, 3px3p kernal window, 1wx1h stride, relu
    x = Dropout(drop)(x)                                                      # Randomly drop out (turn off) 10% of the neurons (Prevent overfitting)
    # Possibly add MaxPooling (will make it less sensitive to position in image).  Camera angle fixed, so may not to be needed

    x = Flatten(name='flattened')(x)                                        # Flatten to 1D (Fully connected)
    x = Dense(100, activation='relu', name="fc_1")(x)                                    # Classify the data into 100 features, make all negatives 0
    x = Dropout(drop)(x)                                                      # Randomly drop out (turn off) 10% of the neurons (Prevent overfitting)
    x = Dense(50, activation='relu', name="fc_2")(x)                                     # Classify the data into 50 features, make all negatives 0
    x = Dropout(drop)(x)                                                      # Randomly drop out 10% of the neurons (Prevent overfitting)
    #categorical output of the angle
    angle_out = Dense(15, activation='softmax', name='angle_out')(x)        # Connect every input with every output and output 15 hidden units. Use Softmax to give percentage. 15 categories and find best one based off percentage 0.0-1.0
    
    #continous output of throttle
    throttle_out = Dense(20, activation='softmax', name='throttle_out')(x)      # Reduce to 1 number, Positive number only
    
    model = Model(inputs=[img_in], outputs=[angle_out, throttle_out])
    return model


def default_n_linear(num_outputs, input_shape):
    from keras.layers import Input, Dense
    from keras.models import Model
    from keras.layers import Convolution2D, MaxPooling2D, Reshape, BatchNormalization
    from keras.layers import Activation, Dropout, Flatten, Cropping2D, Lambda

    drop = 0.1
    
    img_in = Input(shape=input_shape, name='img_in')
    x = img_in
    x = Cropping2D(cropping=((10,0), (0,0)))(x) #trim 10 pixels off top
    #x = Lambda(lambda x: x/127.5 - 1.)(x) # normalize and re-center
    x = Convolution2D(24, (5,5), strides=(2,2), activation='relu', name="conv2d_1")(x)
    x = Dropout(drop)(x)
    x = Convolution2D(32, (5,5), strides=(2,2), activation='relu', name="conv2d_2")(x)
    x = Dropout(drop)(x)
    x = Convolution2D(64, (5,5), strides=(2,2), activation='relu', name="conv2d_3")(x)
    x = Dropout(drop)(x)
    x = Convolution2D(64, (3,3), strides=(1,1), activation='relu', name="conv2d_4")(x)
    x = Dropout(drop)(x)
    x = Convolution2D(64, (3,3), strides=(1,1), activation='relu', name="conv2d_5")(x)
    x = Dropout(drop)(x)
    
    x = Flatten(name='flattened')(x)
    x = Dense(100, activation='relu')(x)
    x = Dropout(drop)(x)
    x = Dense(50, activation='relu')(x)
    x = Dropout(drop)(x)

    outputs = []
    
    for i in range(num_outputs):
        outputs.append(Dense(1, activation='linear', name='n_outputs' + str(i))(x))
        
    model = Model(inputs=[img_in], outputs=outputs)
    
    return model



def default_imu(num_outputs, num_imu_inputs, input_shape):
    '''
    Notes: this model depends on concatenate which failed on keras < 2.0.8
    '''

    from keras.layers import Input, Dense
    from keras.models import Model
    from keras.layers import Convolution2D, MaxPooling2D, Reshape, BatchNormalization
    from keras.layers import Activation, Dropout, Flatten, Cropping2D, Lambda
    from keras.layers.merge import concatenate
    
    img_in = Input(shape=input_shape, name='img_in')
    imu_in = Input(shape=(num_imu_inputs,), name="imu_in")
    
    x = img_in
    x = Cropping2D(cropping=((60,0), (0,0)))(x) #trim 60 pixels off top
    #x = Lambda(lambda x: x/127.5 - 1.)(x) # normalize and re-center
    x = Convolution2D(24, (5,5), strides=(2,2), activation='relu')(x)
    x = Convolution2D(32, (5,5), strides=(2,2), activation='relu')(x)
    x = Convolution2D(64, (3,3), strides=(2,2), activation='relu')(x)
    x = Convolution2D(64, (3,3), strides=(1,1), activation='relu')(x)
    x = Convolution2D(64, (3,3), strides=(1,1), activation='relu')(x)
    x = Flatten(name='flattened')(x)
    x = Dense(100, activation='relu')(x)
    x = Dropout(.1)(x)
    
    y = imu_in
    y = Dense(14, activation='relu')(y)
    y = Dense(14, activation='relu')(y)
    y = Dense(14, activation='relu')(y)
    
    z = concatenate([x, y])
    z = Dense(50, activation='relu')(z)
    z = Dropout(.1)(z)
    z = Dense(50, activation='relu')(z)
    z = Dropout(.1)(z)

    outputs = [] 
    
    for i in range(num_outputs):
        outputs.append(Dense(1, activation='linear', name='out_' + str(i))(z))
        
    model = Model(inputs=[img_in, imu_in], outputs=outputs)
    
    return model


def default_bhv(num_outputs, num_bvh_inputs, input_shape):
    '''
    Notes: this model depends on concatenate which failed on keras < 2.0.8
    '''

    from keras.layers import Input, Dense
    from keras.models import Model
    from keras.layers import Convolution2D, MaxPooling2D, Reshape, BatchNormalization
    from keras.layers import Activation, Dropout, Flatten, Cropping2D, Lambda
    from keras.layers.merge import concatenate
    
    img_in = Input(shape=input_shape, name='img_in')
    bvh_in = Input(shape=(num_bvh_inputs,), name="behavior_in")
    
    x = img_in
    x = Cropping2D(cropping=((60,0), (0,0)))(x) #trim 60 pixels off top
    #x = Lambda(lambda x: x/127.5 - 1.)(x) # normalize and re-center
    x = Convolution2D(24, (5,5), strides=(2,2), activation='relu')(x)
    x = Convolution2D(32, (5,5), strides=(2,2), activation='relu')(x)
    x = Convolution2D(64, (5,5), strides=(2,2), activation='relu')(x)
    x = Convolution2D(64, (3,3), strides=(1,1), activation='relu')(x)
    x = Convolution2D(64, (3,3), strides=(1,1), activation='relu')(x)
    x = Flatten(name='flattened')(x)
    x = Dense(100, activation='relu')(x)
    x = Dropout(.1)(x)
    
    y = bvh_in
    y = Dense(num_bvh_inputs * 2, activation='relu')(y)
    y = Dense(num_bvh_inputs * 2, activation='relu')(y)
    y = Dense(num_bvh_inputs * 2, activation='relu')(y)
    
    z = concatenate([x, y])
    z = Dense(100, activation='relu')(z)
    z = Dropout(.1)(z)
    z = Dense(50, activation='relu')(z)
    z = Dropout(.1)(z)
    
  #categorical output of the angle
    angle_out = Dense(15, activation='softmax', name='angle_out')(z)        # Connect every input with every output and output 15 hidden units. Use Softmax to give percentage. 15 categories and find best one based off percentage 0.0-1.0
    
    #continous output of throttle
    throttle_out = Dense(20, activation='softmax', name='throttle_out')(z)      # Reduce to 1 number, Positive number only
        
    model = Model(inputs=[img_in, bvh_in], outputs=[angle_out, throttle_out])
    
    return model


def default_loc(num_outputs, num_locations, input_shape):
    '''
    Notes: this model depends on concatenate which failed on keras < 2.0.8
    '''

    from keras.layers import Input, Dense
    from keras.models import Model
    from keras.layers import Convolution2D, MaxPooling2D, Reshape, BatchNormalization
    from keras.layers import Activation, Dropout, Flatten, Cropping2D, Lambda
    from keras.layers.merge import concatenate
    from donkeycar.contrib.coordconv.coord import CoordinateChannel2D
    
    drop = 0.5

    img_in = Input(shape=input_shape, name='img_in')
    
    x = img_in
    #x = Cropping2D(cropping=((10,0), (0,0)))(x) #trim 10 pixels off top
    x = Lambda(lambda x: x/127.5 - 1.)(x) # normalize and re-center
    x = CoordinateChannel2D()(x)
    x = Convolution2D(24, (5,5), strides=(2,2), activation='relu', name="conv2d_1")(x)
    x = Dropout(drop)(x)
    x = Convolution2D(32, (5,5), strides=(2,2), activation='relu', name="conv2d_2")(x)
    x = Dropout(drop)(x)
    x = Convolution2D(64, (5,5), strides=(2,2), activation='relu', name="conv2d_3")(x)
    x = Dropout(drop)(x)
    x = Convolution2D(64, (3,3), strides=(2,2), activation='relu', name="conv2d_4")(x)
    x = Dropout(drop)(x)
    x = Convolution2D(64, (3,3), strides=(1,1), activation='relu', name="conv2d_5")(x)
    x = Dropout(drop)(x)
    x = Flatten(name='flattened')(x)
    x = Dense(100, activation='relu')(x)
    x = Dropout(drop)(x)
    
    z = Dense(50, activation='relu')(x)
    z = Dropout(.1)(z)
    
    
    #categorical output of the angle
    angle_out = Dense(15, activation='softmax', name='angle')(z)
    
    #categorical output of throttle
    throttle_out = Dense(20, activation='softmax', name='throttle')(z)

    #categorical output of location
    loc_out = Dense(num_locations, activation='softmax', name='loc')(z)

    #categorical output of lane
    lane_out = Dense(2, activation='softmax', name='lane')(z)

    #model = Model(inputs=[img_in], outputs=[angle_out, throttle_out, loc_out, lane_out])
    model = Model(inputs=[img_in], outputs=[angle_out, throttle_out, loc_out])
    
    return model


class KerasRNN_LSTM(KerasPilot):
    def __init__(self, image_w =160, image_h=120, image_d=3, seq_length=3, num_outputs=2, *args, **kwargs):
        super(KerasRNN_LSTM, self).__init__(*args, **kwargs)
        image_shape = (image_h, image_w, image_d)
        self.model = rnn_lstm(seq_length=seq_length,
            num_outputs=num_outputs,
            image_shape=image_shape)
        self.seq_length = seq_length
        self.image_d = image_d
        self.image_w = image_w
        self.image_h = image_h
        self.img_seq = []
        self.compile()
        self.optimizer = "rmsprop"

    def compile(self):
        self.model.compile(optimizer=self.optimizer,
                  loss='mse')

    def run(self, img_arr):
        if img_arr.shape[2] == 3 and self.image_d == 1:
            img_arr = dk.utils.rgb2gray(img_arr)

        while len(self.img_seq) < self.seq_length:
            self.img_seq.append(img_arr)

        self.img_seq = self.img_seq[1:]
        self.img_seq.append(img_arr)
        
        img_arr = np.array(self.img_seq).reshape(1, self.seq_length, self.image_h, self.image_w, self.image_d )
        outputs = self.model.predict([img_arr])
        steering = outputs[0][0]
        throttle = outputs[0][1]
        return steering, throttle
  

def rnn_lstm(seq_length=3, num_outputs=2, image_shape=(120,160,3)):

    from keras.layers import Input, Dense
    from keras.models import Sequential
    from keras.layers import Convolution2D, MaxPooling2D, Reshape, BatchNormalization
    from keras.layers import Activation, Dropout, Flatten, Cropping2D, Lambda
    from keras.layers.merge import concatenate
    from keras.layers import LSTM
    from keras.layers.wrappers import TimeDistributed as TD

    img_seq_shape = (seq_length,) + image_shape   
    img_in = Input(batch_shape = img_seq_shape, name='img_in')
    
    x = Sequential()
    x.add(TD(Cropping2D(cropping=((60,0), (0,0))), input_shape=img_seq_shape )) #trim 60 pixels off top
    x.add(TD(Convolution2D(24, (5,5), strides=(2,2), activation='relu')))
    x.add(TD(Convolution2D(32, (5,5), strides=(2,2), activation='relu')))
    x.add(TD(Convolution2D(32, (3,3), strides=(2,2), activation='relu')))
    x.add(TD(Convolution2D(32, (3,3), strides=(1,1), activation='relu')))
    x.add(TD(MaxPooling2D(pool_size=(2, 2))))
    x.add(TD(Flatten(name='flattened')))
    x.add(TD(Dense(100, activation='relu')))
    x.add(TD(Dropout(.1)))
      
    x.add(LSTM(128, return_sequences=True, name="LSTM_seq"))
    x.add(Dropout(.1))
    x.add(LSTM(128, return_sequences=False, name="LSTM_out"))
    x.add(Dropout(.1))
    x.add(Dense(128, activation='relu'))
    x.add(Dropout(.1))
    x.add(Dense(64, activation='relu'))
    x.add(Dense(10, activation='relu'))
    x.add(Dense(num_outputs, activation='linear', name='model_outputs'))
    
    return x


class Keras3D_CNN(KerasPilot):
    def __init__(self, image_w =160, image_h=120, image_d=3, seq_length=20, num_outputs=2, *args, **kwargs):
        super(Keras3D_CNN, self).__init__(*args, **kwargs)
        self.model = build_3d_cnn(w=image_w, h=image_h, d=image_d, s=seq_length, num_outputs=num_outputs)
        self.seq_length = seq_length
        self.image_d = image_d
        self.image_w = image_w
        self.image_h = image_h
        self.img_seq = []
        self.compile()

    def compile(self):
        self.model.compile(loss='mean_squared_error', optimizer=self.optimizer, metrics=['accuracy'])

    def run(self, img_arr):

        if img_arr.shape[2] == 3 and self.image_d == 1:
            img_arr = dk.utils.rgb2gray(img_arr)

        while len(self.img_seq) < self.seq_length:
            self.img_seq.append(img_arr)

        self.img_seq = self.img_seq[1:]
        self.img_seq.append(img_arr)
        
        img_arr = np.array(self.img_seq).reshape(1, self.seq_length, self.image_h, self.image_w, self.image_d )
        outputs = self.model.predict([img_arr])
        steering = outputs[0][0]
        throttle = outputs[0][1]
        return steering, throttle


def build_3d_cnn(w, h, d, s, num_outputs):
    from keras.layers import Input, Dense
    from keras.models import Sequential
    from keras.layers import Conv3D, MaxPooling3D, Reshape, BatchNormalization
    from keras.layers import Activation, Dropout, Flatten, Cropping3D

    #Credit: https://github.com/jessecha/DNRacing/blob/master/3D_CNN_Model/model.py
    '''
        w : width
        h : height
        d : depth
        s : n_stacked
    '''
    input_shape=(s, h, w, d)

    model = Sequential()
    #First layer
    model.add(Cropping3D(cropping=((0,0), (50,10), (0,0)), input_shape=input_shape) ) #trim pixels off top
    
    # Second layer
    model.add(Conv3D(
        filters=16, kernel_size=(3,3,3), strides=(1,3,3),
        data_format='channels_last', padding='same')
    )
    model.add(Activation('relu'))
    model.add(MaxPooling3D(
        pool_size=(1,2,2), strides=(1,2,2), padding='valid', data_format=None)
    )
    # Third layer
    model.add(Conv3D(
        filters=32, kernel_size=(3,3,3), strides=(1,1,1),
        data_format='channels_last', padding='same')
    )
    model.add(Activation('relu'))
    model.add(MaxPooling3D(
        pool_size=(1, 2, 2), strides=(1,2,2), padding='valid', data_format=None)
    )
    # Fourth layer
    model.add(Conv3D(
        filters=64, kernel_size=(3,3,3), strides=(1,1,1),
        data_format='channels_last', padding='same')
    )
    model.add(Activation('relu'))
    model.add(MaxPooling3D(
        pool_size=(1,2,2), strides=(1,2,2), padding='valid', data_format=None)
    )
    # Fifth layer
    model.add(Conv3D(
        filters=128, kernel_size=(3,3,3), strides=(1,1,1),
        data_format='channels_last', padding='same')
    )
    model.add(Activation('relu'))
    model.add(MaxPooling3D(
        pool_size=(1,2,2), strides=(1,2,2), padding='valid', data_format=None)
    )
    # Fully connected layer
    model.add(Flatten())

    model.add(Dense(256))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(Dropout(0.5))

    model.add(Dense(256))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(Dropout(0.5))

    model.add(Dense(num_outputs))
    #model.add(Activation('tanh'))

    return model
