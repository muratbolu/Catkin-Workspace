; Auto-generated. Do not edit!


(cl:in-package ros_exercises-srv)


;//! \htmlinclude find_min_array-request.msg.html

(cl:defclass <find_min_array-request> (roslisp-msg-protocol:ros-message)
  ((array
    :reader array
    :initarg :array
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass find_min_array-request (<find_min_array-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <find_min_array-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'find_min_array-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_exercises-srv:<find_min_array-request> is deprecated: use ros_exercises-srv:find_min_array-request instead.")))

(cl:ensure-generic-function 'array-val :lambda-list '(m))
(cl:defmethod array-val ((m <find_min_array-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_exercises-srv:array-val is deprecated.  Use ros_exercises-srv:array instead.")
  (array m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <find_min_array-request>) ostream)
  "Serializes a message object of type '<find_min_array-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'array))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'array))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <find_min_array-request>) istream)
  "Deserializes a message object of type '<find_min_array-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'array) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'array)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<find_min_array-request>)))
  "Returns string type for a service object of type '<find_min_array-request>"
  "ros_exercises/find_min_arrayRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'find_min_array-request)))
  "Returns string type for a service object of type 'find_min_array-request"
  "ros_exercises/find_min_arrayRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<find_min_array-request>)))
  "Returns md5sum for a message object of type '<find_min_array-request>"
  "c14a13c8d67ff0e22069dfcdf7c3d804")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'find_min_array-request)))
  "Returns md5sum for a message object of type 'find_min_array-request"
  "c14a13c8d67ff0e22069dfcdf7c3d804")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<find_min_array-request>)))
  "Returns full string definition for message of type '<find_min_array-request>"
  (cl:format cl:nil "float32[] array~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'find_min_array-request)))
  "Returns full string definition for message of type 'find_min_array-request"
  (cl:format cl:nil "float32[] array~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <find_min_array-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <find_min_array-request>))
  "Converts a ROS message object to a list"
  (cl:list 'find_min_array-request
    (cl:cons ':array (array msg))
))
;//! \htmlinclude find_min_array-response.msg.html

(cl:defclass <find_min_array-response> (roslisp-msg-protocol:ros-message)
  ((min
    :reader min
    :initarg :min
    :type cl:float
    :initform 0.0))
)

(cl:defclass find_min_array-response (<find_min_array-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <find_min_array-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'find_min_array-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_exercises-srv:<find_min_array-response> is deprecated: use ros_exercises-srv:find_min_array-response instead.")))

(cl:ensure-generic-function 'min-val :lambda-list '(m))
(cl:defmethod min-val ((m <find_min_array-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_exercises-srv:min-val is deprecated.  Use ros_exercises-srv:min instead.")
  (min m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <find_min_array-response>) ostream)
  "Serializes a message object of type '<find_min_array-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <find_min_array-response>) istream)
  "Deserializes a message object of type '<find_min_array-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<find_min_array-response>)))
  "Returns string type for a service object of type '<find_min_array-response>"
  "ros_exercises/find_min_arrayResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'find_min_array-response)))
  "Returns string type for a service object of type 'find_min_array-response"
  "ros_exercises/find_min_arrayResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<find_min_array-response>)))
  "Returns md5sum for a message object of type '<find_min_array-response>"
  "c14a13c8d67ff0e22069dfcdf7c3d804")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'find_min_array-response)))
  "Returns md5sum for a message object of type 'find_min_array-response"
  "c14a13c8d67ff0e22069dfcdf7c3d804")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<find_min_array-response>)))
  "Returns full string definition for message of type '<find_min_array-response>"
  (cl:format cl:nil "float32 min~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'find_min_array-response)))
  "Returns full string definition for message of type 'find_min_array-response"
  (cl:format cl:nil "float32 min~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <find_min_array-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <find_min_array-response>))
  "Converts a ROS message object to a list"
  (cl:list 'find_min_array-response
    (cl:cons ':min (min msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'find_min_array)))
  'find_min_array-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'find_min_array)))
  'find_min_array-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'find_min_array)))
  "Returns string type for a service object of type '<find_min_array>"
  "ros_exercises/find_min_array")