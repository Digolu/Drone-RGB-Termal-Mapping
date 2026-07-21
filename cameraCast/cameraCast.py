import cv2
from pypylon import pylon
from flask import Flask, Response

app = Flask(__name__)

tlf = pylon.TlFactory.GetInstance()
camera = pylon.InstantCamera(tlf.CreateFirstDevice())
camera.Open()
print("Modelo:", camera.GetDeviceInfo().GetModelName())
print("PixelFormat:", camera.PixelFormat.GetValue())

camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

def generate():
    while camera.IsGrabbing():
        grab = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grab.GrabSucceeded():
            img_yuv = grab.Array
            img_bgr = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR_YUYV)
            ok, jpeg = cv2.imencode('.jpg', img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                frame = jpeg.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        grab.Release()

@app.route('/video')
def video():
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        camera.StopGrabbing()
        camera.Close()