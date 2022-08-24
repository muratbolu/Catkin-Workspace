; Auto-generated. Do not edit!


(cl:in-package ros_exercises-srv)


;//! \htmlinclude compute_statistics-request.msg.html

(cl:defclass <compute_statistics-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass compute_statistics-request (<compute_statistics-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <compute_statistics-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'compute_statistics-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_exercises-srv:<compute_statistics-request> is deprecated: use ros_exercises-srv:compute_statistics-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <compute_statistics-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_exercises-srv:input-val is deprecated.  Use ros_exercises-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <compute_statistics-request>) ostream)
  "Serializes a message object of type '<compute_statistics-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'input))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'input))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <compute_statistics-request>) istream)
  "Deserializes a message object of type '<compute_statistics-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'input) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'input)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<compute_statistics-request>)))
  "Returns string type for a service object of type '<compute_statistics-request>"
  "ros_exercises/compute_statisticsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'compute_statistics-request)))
  "Returns string type for a service object of type 'compute_statistics-request"
  "ros_exercises/compute_statisticsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<compute_statistics-request>)))
  "Returns md5sum for a message object of type '<compute_statistics-request>"
  "89a0f83e0765b462301612617aaed4ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'compute_statistics-request)))
  "Returns md5sum for a message object of type 'compute_statistics-request"
  "89a0f83e0765b462301612617aaed4ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<compute_statistics-request>)))
  "Returns full string definition for message of type '<compute_statistics-request>"
  (cl:format cl:nil "float32[] input~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'compute_statistics-request)))
  "Returns full string definition for message of type 'compute_statistics-request"
  (cl:format cl:nil "float32[] input~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <compute_statistics-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'input) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <compute_statistics-request>))
  "Converts a ROS message object to a list"
  (cl:list 'compute_statistics-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude compute_statistics-response.msg.html

(cl:defclass <compute_statistics-response> (roslisp-msg-protocol:ros-message)
  ((median
    :reader median
    :initarg :median
    :type cl:float
    :initform 0.0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:float
    :initform 0.0)
   (mean
    :reader mean
    :initarg :mean
    :type cl:float
    :initform 0.0))
)

(cl:defclass compute_statistics-response (<compute_statistics-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <compute_statistics-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'compute_statistics-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_exercises-srv:<compute_statistics-response> is deprecated: use ros_exercises-srv:compute_statistics-response instead.")))

(cl:ensure-generic-function 'median-val :lambda-list '(m))
(cl:defmethod median-val ((m <compute_statistics-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_exercises-srv:median-val is deprecated.  Use ros_exercises-srv:median instead.")
  (median m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <compute_statistics-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_exercises-srv:mode-val is deprecated.  Use ros_exercises-srv:mode instead.")
  (mode m))

(cl:ensure-generic-function 'mean-val :lambda-list '(m))
(cl:defmethod mean-val ((m <compute_statistics-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_exercises-srv:mean-val is deprecated.  Use ros_exercises-srv:mean instead.")
  (mean m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <compute_statistics-response>) ostream)
  "Serializes a message object of type '<compute_statistics-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'median))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mean))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <compute_statistics-response>) istream)
  "Deserializes a message object of type '<compute_statistics-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'median) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mode) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mean) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<compute_statistics-response>)))
  "Returns string type for a service object of type '<compute_statistics-response>"
  "ros_exercises/compute_statisticsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'compute_statistics-response)))
  "Returns string type for a service object of type 'compute_statistics-response"
  "ros_exercises/compute_statisticsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<compute_statistics-response>)))
  "Returns md5sum for a message object of type '<compute_statistics-response>"
  "89a0f83e0765b462301612617aaed4ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'compute_statistics-response)))
  "Returns md5sum for a message object of type 'compute_statistics-response"
  "89a0f83e0765b462301612617aaed4ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<compute_statistics-response>)))
  "Returns full string definition for message of type '<compute_statistics-response>"
  (cl:format cl:nil "float32 median~%float32 mode~%float32 mean~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'compute_statistics-response)))
  "Returns full string definition for message of type 'compute_statistics-response"
  (cl:format cl:nil "float32 median~%float32 mode~%float32 mean~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <compute_statistics-response>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <compute_statistics-response>))
  "Converts a ROS message object to a list"
  (cl:list 'compute_statistics-response
    (cl:cons ':median (median msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':mean (mean msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'compute_statistics)))
  'compute_statistics-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'compute_statistics)))
  'compute_statistics-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'compute_statistics)))
  "Returns string type for a service object of type '<compute_statistics>"
  "ros_exercises/compute_statistics")