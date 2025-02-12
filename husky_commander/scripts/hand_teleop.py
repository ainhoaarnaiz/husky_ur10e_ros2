import cv2 as cv
import mediapipe as mp

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

vid = cv.VideoCapture(0) #change number to select webcam

def getHandMove(hand_landmarks):
	
	landmarks = hand_landmarks.landmark
	
	if all([landmarks[i].y < landmarks[i + 3].y for i in range(5, 20, 4)]): 
		return "rock"  # i: 5 and 8, 9 and 12, 13 and 16...
	elif landmarks[13].y < landmarks[16].y and landmarks[17].y < landmarks[20].y and landmarks[5].y > landmarks[8].y and landmarks[9].y > landmarks[12].y: 
		return "scissors"
	elif all([landmarks[i].y > landmarks[i + 3].y for i in range(5, 20, 4)]): 
		return "paper"
	else: 
		return "unknown"


with mp_hands.Hands(model_complexity=0,
					max_num_hands=1, 
					min_detection_confidence=0.5, 
					min_tracking_confidence=0.5) as hands:
	while True:
		ret, frame = vid.read()
		if not ret or frame is None: break

		frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
		results = hands.process(frame)
		frame =cv.cvtColor(frame, cv.COLOR_RGB2BGR)

		hls = results.multi_hand_landmarks

		if hls:
			for hand_landmarks in hls:
				mp_drawing.draw_landmarks(frame, 
							  			  hand_landmarks, 
										  mp_hands.HAND_CONNECTIONS,
										  mp_drawing_styles.get_default_hand_landmarks_style(),
										  mp_drawing_styles.get_default_hand_connections_style())
				
		frame = cv.flip(frame, 1)

		if hls is None: hand_move = "no hand detected"
		else: hand_move = getHandMove(hls[0])

		cv.putText(frame, "Hand Move: " + hand_move, (50,50), cv.FONT_HERSHEY_PLAIN, 2, (0,255,255), 2, cv.LINE_AA)

		cv.imshow('frame', frame)

		if cv.waitKey(1) & 0xFF == ord('q'): break

vid.release()
cv.destroyAllWindows()