
import os
from time import time
import thread

class EmotionManager():
    def __init__(self):

	self.wait_different_emotions = 0 #20 #sec
	self.wait_same_emotions = 180 #sec

	self.last_emotion = ''
	self.last_emotion_time = time()
	self.last_phrase = ''

	pass

    def emotion_handler(self, emotion):
	if self.last_emotion is '': #first time
	    pass
	elif emotion is not self.last_emotion:
	    if time()-self.last_emotion_time < self.wait_different_emotions:
	        return
	elif emotion is self.last_emotion:
	    if time()-self.last_emotion_time < self.wait_same_emotions:
	        return
	
	self.last_emotion = emotion
	self.last_emotion_time = time()

	if   emotion=='angry'    : phrase = self.angry_response()	
	elif emotion=='happy'    : phrase = self.happy_response()	
	elif emotion=='neutral'  : phrase = self.neutral_response()	
	elif emotion=='sad'      : phrase = self.sad_response()	
	elif emotion=='surprise' : phrase = self.surprised_response()	

	self.last_phrase = phrase
	return phrase

    def angry_response(self):
	mp3_file = 'mp3s/angry1.mp3'
	thread.start_new_thread( self.play_audio, (mp3_file, ) )
	return 'Deep breaths, Phil, or you might hurt yourself.'

    def happy_response(self):
	mp3_file = 'mp3s/happy1.mp3'
	thread.start_new_thread( self.play_audio, (mp3_file, ) )
	return 'You look so happy! I\'m going to take a picture!' 

    def neutral_response(self):
	mp3_file = 'mp3s/neutral1.mp3'
	thread.start_new_thread( self.play_audio, (mp3_file, ) )
	return 'How about those DETROIT Lions?'

    def sad_response(self):
	mp3_file = 'mp3s/sad1.mp3'
	thread.start_new_thread( self.play_audio, (mp3_file, ) )
	return 'Hi Phil. You look sad - want to talk about it?'

    def surprised_response(self):
	mp3_file = 'mp3s/surprised1.mp3'
	thread.start_new_thread( self.play_audio, (mp3_file, ) )
	return 'Surprised to see me?' 

    def play_audio(self, mp3_file):
        os.system('ffplay '+mp3_file+' -nodisp -nostats -hide_banner -autoexit')
	

if __name__=='__main__':
    e = EmotionManager()

    e.angry_response() 
    e.happy_response() 
    e.neutral_response() 
    e.sad_response() 

