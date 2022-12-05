import cv2
import mediapipe as mp

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

# everything in meter
ref_real_distance = 1.6
ref_real_size = 0.4
ref_pixel_size = 0.0006433203816413879
focal_length = (ref_pixel_size * ref_real_distance) / ref_real_size


def distance_finder(width_in_frame):
    return (ref_real_size * focal_length) / width_in_frame


cap = cv2.VideoCapture(0)
cap_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
cap_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
cap_fps = int(cap.get(cv2.CAP_PROP_FPS))
if cap_fps == 0:
    exit("Couldn't get camera")
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            # If loading a video, use 'break' instead of 'continue'.
            continue

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        # Flip the image horizontally for a later selfie-view display, and convert
        # the BGR image to RGB.
        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        results = pose.process(image)

        # Draw the pose annotation on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                  landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
        # mp_drawing.plot_landmarks(results.pose_world_landmarks, mp_pose.POSE_CONNECTIONS)

        if results.pose_world_landmarks:
            hips = 0.5 * (results.pose_world_landmarks.landmark[
                              mp_pose.PoseLandmark.RIGHT_HIP].z + results.pose_world_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP].z)
            image = cv2.putText(image, f"Distance: {distance_finder(hips)}", (50, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0))
            print(distance_finder(hips))

        cv2.imshow('MediaPipe Pose', image)
        if cv2.waitKey(5) & 0xFF == 27:
            break

cap.release()
cv2.destroyAllWindows()
