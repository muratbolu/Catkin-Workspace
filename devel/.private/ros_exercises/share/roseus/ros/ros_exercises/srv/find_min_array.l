;; Auto-generated. Do not edit!


(when (boundp 'ros_exercises::find_min_array)
  (if (not (find-package "ROS_EXERCISES"))
    (make-package "ROS_EXERCISES"))
  (shadow 'find_min_array (find-package "ROS_EXERCISES")))
(unless (find-package "ROS_EXERCISES::FIND_MIN_ARRAY")
  (make-package "ROS_EXERCISES::FIND_MIN_ARRAY"))
(unless (find-package "ROS_EXERCISES::FIND_MIN_ARRAYREQUEST")
  (make-package "ROS_EXERCISES::FIND_MIN_ARRAYREQUEST"))
(unless (find-package "ROS_EXERCISES::FIND_MIN_ARRAYRESPONSE")
  (make-package "ROS_EXERCISES::FIND_MIN_ARRAYRESPONSE"))

(in-package "ROS")





(defclass ros_exercises::find_min_arrayRequest
  :super ros::object
  :slots (_array ))

(defmethod ros_exercises::find_min_arrayRequest
  (:init
   (&key
    ((:array __array) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _array __array)
   self)
  (:array
   (&optional __array)
   (if __array (setq _array __array)) _array)
  (:serialization-length
   ()
   (+
    ;; float32[] _array
    (* 4    (length _array)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _array
     (write-long (length _array) s)
     (dotimes (i (length _array))
       (sys::poke (elt _array i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _array
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _array (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _array i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass ros_exercises::find_min_arrayResponse
  :super ros::object
  :slots (_min ))

(defmethod ros_exercises::find_min_arrayResponse
  (:init
   (&key
    ((:min __min) 0.0)
    )
   (send-super :init)
   (setq _min (float __min))
   self)
  (:min
   (&optional __min)
   (if __min (setq _min __min)) _min)
  (:serialization-length
   ()
   (+
    ;; float32 _min
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _min
       (sys::poke _min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _min
     (setq _min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ros_exercises::find_min_array
  :super ros::object
  :slots ())

(setf (get ros_exercises::find_min_array :md5sum-) "c14a13c8d67ff0e22069dfcdf7c3d804")
(setf (get ros_exercises::find_min_array :datatype-) "ros_exercises/find_min_array")
(setf (get ros_exercises::find_min_array :request) ros_exercises::find_min_arrayRequest)
(setf (get ros_exercises::find_min_array :response) ros_exercises::find_min_arrayResponse)

(defmethod ros_exercises::find_min_arrayRequest
  (:response () (instance ros_exercises::find_min_arrayResponse :init)))

(setf (get ros_exercises::find_min_arrayRequest :md5sum-) "c14a13c8d67ff0e22069dfcdf7c3d804")
(setf (get ros_exercises::find_min_arrayRequest :datatype-) "ros_exercises/find_min_arrayRequest")
(setf (get ros_exercises::find_min_arrayRequest :definition-)
      "float32[] array
---
float32 min

")

(setf (get ros_exercises::find_min_arrayResponse :md5sum-) "c14a13c8d67ff0e22069dfcdf7c3d804")
(setf (get ros_exercises::find_min_arrayResponse :datatype-) "ros_exercises/find_min_arrayResponse")
(setf (get ros_exercises::find_min_arrayResponse :definition-)
      "float32[] array
---
float32 min

")



(provide :ros_exercises/find_min_array "c14a13c8d67ff0e22069dfcdf7c3d804")


