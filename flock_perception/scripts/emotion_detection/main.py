from keras.models import load_model
from time import sleep
from keras.preprocessing.image import img_to_array
from keras.preprocessing import image
import cv2
import numpy as np

from emotion_manager import EmotionManager
em = EmotionManager()

face_classifier = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
classifier =load_model('Emotion_little_vgg.h5')

class_labels = ['angry','happy','neutral','sad','surprise']
class_label_memory = {'angry':0, 'happy':0, 'neutral':0, 'sad':0, 'surprise':0, 'noface':0}

#cap = cv2.VideoCapture('/home/lxu9/gazebo_ws/src/drone/flock/flock_perception/rosbags/facial_expressions_air.mp4')
#cap = cv2.VideoCapture('/home/lxu9/gazebo_ws/src/drone/flock/flock_perception/rosbags/debug_image_follower.mp4')
cap = cv2.VideoCapture('/home/lxu9/gazebo_ws/src/drone/flock/flock_perception/rosbags/image_raw.mp4')
#cap = cv2.VideoCapture(0)



while True:
    # Grab a single frame of video
    ret, frame = cap.read()
    labels = []
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    faces = face_classifier.detectMultiScale(gray,1.3,5)

    for (x,y,w,h) in faces:
        cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        roi_gray = gray[y:y+h,x:x+w]
        roi_gray = cv2.resize(roi_gray,(48,48),interpolation=cv2.INTER_AREA)
        # rect,face,image = face_detector(frame)


        if np.sum([roi_gray])!=0:
            roi = roi_gray.astype('float')/255.0
            roi = img_to_array(roi)
            roi = np.expand_dims(roi,axis=0)

            # make a prediction on the ROI, then lookup the class
            preds = classifier.predict(roi)[0]
            label=class_labels[preds.argmax()]
            label_position = (x,y)
            cv2.putText(frame,label,label_position,cv2.FONT_HERSHEY_SIMPLEX,2,(0,255,0),3)
        else:
            cv2.putText(frame,'No Face Found',(20,60),cv2.FONT_HERSHEY_SIMPLEX,2,(0,255,0),3)
	    label='noface'

	class_label_memory[label] += 3 
	for label in class_label_memory:
	    if class_label_memory[label] > 120:
		class_label_memory[label] = 0
	        trigger_str = 'Triggered '+label+' response'
        	cv2.putText(frame,trigger_str, (20,300),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),3)
		phrase = em.emotion_handler(label)
		print(phrase)

	    class_label_memory[label] -= 1
	    if class_label_memory[label] < 0:
		class_label_memory[label] = 0
	label_count_str  = 'Ang:'+str(class_label_memory['angry'])
	label_count_str1 = 'Hap:'+str(class_label_memory['happy'])
	label_count_str2 = 'Neu:'+str(class_label_memory['neutral'])
	label_count_str3 = 'Sad:'+str(class_label_memory['sad'])
	label_count_str4 = 'Sur:'+str(class_label_memory['surprise'])
	label_count_str5 = 'NoF:'+str(class_label_memory['noface'])
        cv2.putText(frame,label_count_str, (20,60),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),3)
        cv2.putText(frame,label_count_str1,(20,100),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),3)
        cv2.putText(frame,label_count_str2,(20,140),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),3)
        cv2.putText(frame,label_count_str3,(20,180),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),3)
        cv2.putText(frame,label_count_str4,(20,220),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),3)
        cv2.putText(frame,label_count_str5,(20,260),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),3)

	last_strong_emotion = em.last_emotion
	last_phrase = em.last_phrase
        cv2.putText(frame,last_strong_emotion,(20,300),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),3)
        cv2.putText(frame,last_phrase,(20,450),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),3)
 

    cv2.imshow('Emotion Detector',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

if KeyboardInterrupt:
    cap.release()
    cv2.destroyAllWindows()


