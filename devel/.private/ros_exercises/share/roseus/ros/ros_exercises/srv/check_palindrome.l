;; Auto-generated. Do not edit!


(when (boundp 'ros_exercises::check_palindrome)
  (if (not (find-package "ROS_EXERCISES"))
    (make-package "ROS_EXERCISES"))
  (shadow 'check_palindrome (find-package "ROS_EXERCISES")))
(unless (find-package "ROS_EXERCISES::CHECK_PALINDROME")
  (make-package "ROS_EXERCISES::CHECK_PALINDROME"))
(unless (find-package "ROS_EXERCISES::CHECK_PALINDROMEREQUEST")
  (make-package "ROS_EXERCISES::CHECK_PALINDROMEREQUEST"))
(unless (find-package "ROS_EXERCISES::CHECK_PALINDROMERESPONSE")
  (make-package "ROS_EXERCISES::CHECK_PALINDROMERESPONSE"))

(in-package "ROS")





(defclass ros_exercises::check_palindromeRequest
  :super ros::object
  :slots (_input ))

(defmethod ros_exercises::check_palindromeRequest
  (:init
   (&key
    ((:input __input) "")
    )
   (send-super :init)
   (setq _input (string __input))
   self)
  (:input
   (&optional __input)
   (if __input (setq _input __input)) _input)
  (:serialization-length
   ()
   (+
    ;; string _input
    4 (length _input)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _input
       (write-long (length _input) s) (princ _input s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _input
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _input (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ros_exercises::check_palindromeResponse
  :super ros::object
  :slots (_bit ))

(defmethod ros_exercises::check_palindromeResponse
  (:init
   (&key
    ((:bit __bit) 0)
    )
   (send-super :init)
   (setq _bit (round __bit))
   self)
  (:bit
   (&optional __bit)
   (if __bit (setq _bit __bit)) _bit)
  (:serialization-length
   ()
   (+
    ;; int8 _bit
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _bit
       (write-byte _bit s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _bit
     (setq _bit (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _bit 127) (setq _bit (- _bit 256)))
   ;;
   self)
  )

(defclass ros_exercises::check_palindrome
  :super ros::object
  :slots ())

(setf (get ros_exercises::check_palindrome :md5sum-) "9435240a40d8ae413a3b7925d764bbcf")
(setf (get ros_exercises::check_palindrome :datatype-) "ros_exercises/check_palindrome")
(setf (get ros_exercises::check_palindrome :request) ros_exercises::check_palindromeRequest)
(setf (get ros_exercises::check_palindrome :response) ros_exercises::check_palindromeResponse)

(defmethod ros_exercises::check_palindromeRequest
  (:response () (instance ros_exercises::check_palindromeResponse :init)))

(setf (get ros_exercises::check_palindromeRequest :md5sum-) "9435240a40d8ae413a3b7925d764bbcf")
(setf (get ros_exercises::check_palindromeRequest :datatype-) "ros_exercises/check_palindromeRequest")
(setf (get ros_exercises::check_palindromeRequest :definition-)
      "string input
---
int8 bit

")

(setf (get ros_exercises::check_palindromeResponse :md5sum-) "9435240a40d8ae413a3b7925d764bbcf")
(setf (get ros_exercises::check_palindromeResponse :datatype-) "ros_exercises/check_palindromeResponse")
(setf (get ros_exercises::check_palindromeResponse :definition-)
      "string input
---
int8 bit

")



(provide :ros_exercises/check_palindrome "9435240a40d8ae413a3b7925d764bbcf")


