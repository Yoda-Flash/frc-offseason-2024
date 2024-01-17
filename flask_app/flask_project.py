# an object of WSGI application  
from __future__ import print_function
from webcam import WebcamVideoStream
from fps import FPS
import argparse
import imutils
import cv2
from flask import Flask, Response, render_template
import cv2
import apriltag  

ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type=int, default=100,
	help="# of frames to loop over for FPS test")
ap.add_argument("-d", "--display", type=int, default=-1,
	help="Whether or not frames should be displayed")
args = vars(ap.parse_args())

app = Flask(__name__)   # Flask constructor
fps = FPS().start() 
vid = cv2.VideoCapture(0)
vs = WebcamVideoStream(src=0).start()
def gen():	
	while True:
		# success, frame = vid.read()
		success, frame = vid.read()
		if not success:
			break
		else:
			while fps._numFrames < args["num_frames"]:
	# grab the frame from the stream and resize it to have a maximum
	# width of 400 pixels
				frame = imutils.resize(frame, width=400)
				fps.update()
			fps.stop()
			print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
			print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

			# loop over some frames...this time using the threaded stream
			print("[INFO] sampling THREADED frames from webcam...")
			while fps._numFrames < args["num_frames"]:
	# grab the frame from the threaded video stream and resize it
	# to have a maximum width of 400 pixels
				frame = vs.read()
				frame = imutils.resize(frame, width=400)

				fps.update()
# stop the timer and display FPS information
			fps.stop()
			print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
			print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

			gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
			options = apriltag.DetectorOptions(families="tag36h11")
			detector = apriltag.Detector(options)
			results = detector.detect(gray)
			print("[INFO] {} total AprilTags detected".format(len(results)))


		# loop over the AprilTag detection results
			for r in results:
		# extract the bounding box (x, y)-coordinates for the AprilTag
		# and convert each of the (x, y)-coordinate pairs to integers
				(ptA, ptB, ptC, ptD) = r.corners
				ptB = (int(ptB[0]), int(ptB[1]))
				ptC = (int(ptC[0]), int(ptC[1]))
				ptD = (int(ptD[0]), int(ptD[1]))
				ptA = (int(ptA[0]), int(ptA[1]))
		# draw the bounding box of the AprilTag detection
				cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
				cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
				cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
				cv2.line(frame, ptD, ptA, (0, 255, 0), 2)
		# draw the center (x, y)-coordinates of the AprilTag
				(cX, cY) = (int(r.center[0]), int(r.center[1]))
				cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
		# draw the tag family on the image
				tagFamily = r.tag_family.decode("utf-8")
				cv2.putText(frame, tagFamily, (ptA[0], ptA[1] - 15),
				cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
				print("[INFO] tag family: {}".format(tagFamily))
	# show the output image after AprilTag detection
			# if args["display"] > 0:
			ret, buffer = cv2.imencode('.jpg', frame)
			frame = buffer.tobytes()
			yield (b'--frame\r\n'
				b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result

				
		
# A decorator used to tell the application 
# which URL is associated function 
@app.route('/')       
def index():
		# Return the result on the web
	return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame') 

if __name__=='__main__': 
	app.run(debug = True)
   