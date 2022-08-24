;; Auto-generated. Do not edit!


(when (boundp 'ros_exercises::compute_statistics)
  (if (not (find-package "ROS_EXERCISES"))
    (make-package "ROS_EXERCISES"))
  (shadow 'compute_statistics (find-package "ROS_EXERCISES")))
(unless (find-package "ROS_EXERCISES::COMPUTE_STATISTICS")
  (make-package "ROS_EXERCISES::COMPUTE_STATISTICS"))
(unless (find-package "ROS_EXERCISES::COMPUTE_STATISTICSREQUEST")
  (make-package "ROS_EXERCISES::COMPUTE_STATISTICSREQUEST"))
(unless (find-package "ROS_EXERCISES::COMPUTE_STATISTICSRESPONSE")
  (make-package "ROS_EXERCISES::COMPUTE_STATISTICSRESPONSE"))

(in-package "ROS")





(defclass ros_exercises::compute_statisticsRequest
  :super ros::object
  :slots (_input ))

(defmethod ros_exercises::compute_statisticsRequest
  (:init
   (&key
    ((:input __input) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _input __input)
   self)
  (:input
   (&optional __input)
   (if __input (setq _input __input)) _input)
  (:serialization-length
   ()
   (+
    ;; float32[] _input
    (* 4    (length _input)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _input
     (write-long (length _input) s)
     (dotimes (i (length _input))
       (sys::poke (elt _input i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _input
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _input (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _input i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass ros_exercises::compute_statisticsResponse
  :super ros::object
  :slots (_median _mode _mean ))

(defmethod ros_exercises::compute_statisticsResponse
  (:init
   (&key
    ((:median __median) 0.0)
    ((:mode __mode) 0.0)
    ((:mean __mean) 0.0)
    )
   (send-super :init)
   (setq _median (float __median))
   (setq _mode (float __mode))
   (setq _mean (float __mean))
   self)
  (:median
   (&optional __median)
   (if __median (setq _median __median)) _median)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:mean
   (&optional __mean)
   (if __mean (setq _mean __mean)) _mean)
  (:serialization-length
   ()
   (+
    ;; float32 _median
    4
    ;; float32 _mode
    4
    ;; float32 _mean
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _median
       (sys::poke _median (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _mode
       (sys::poke _mode (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _mean
       (sys::poke _mean (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _median
     (setq _median (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _mode
     (setq _mode (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _mean
     (setq _mean (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ros_exercises::compute_statistics
  :super ros::object
  :slots ())

(setf (get ros_exercises::compute_statistics :md5sum-) "89a0f83e0765b462301612617aaed4ac")
(setf (get ros_exercises::compute_statistics :datatype-) "ros_exercises/compute_statistics")
(setf (get ros_exercises::compute_statistics :request) ros_exercises::compute_statisticsRequest)
(setf (get ros_exercises::compute_statistics :response) ros_exercises::compute_statisticsResponse)

(defmethod ros_exercises::compute_statisticsRequest
  (:response () (instance ros_exercises::compute_statisticsResponse :init)))

(setf (get ros_exercises::compute_statisticsRequest :md5sum-) "89a0f83e0765b462301612617aaed4ac")
(setf (get ros_exercises::compute_statisticsRequest :datatype-) "ros_exercises/compute_statisticsRequest")
(setf (get ros_exercises::compute_statisticsRequest :definition-)
      "float32[] input
---
float32 median
float32 mode
float32 mean

")

(setf (get ros_exercises::compute_statisticsResponse :md5sum-) "89a0f83e0765b462301612617aaed4ac")
(setf (get ros_exercises::compute_statisticsResponse :datatype-) "ros_exercises/compute_statisticsResponse")
(setf (get ros_exercises::compute_statisticsResponse :definition-)
      "float32[] input
---
float32 median
float32 mode
float32 mean

")



(provide :ros_exercises/compute_statistics "89a0f83e0765b462301612617aaed4ac")


