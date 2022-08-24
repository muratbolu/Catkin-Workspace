; Auto-generated. Do not edit!


(cl:in-package ros_exercises-msg)


;//! \htmlinclude primes_list.msg.html

(cl:defclass <primes_list> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass primes_list (<primes_list>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <primes_list>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'primes_list)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_exercises-msg:<primes_list> is deprecated: use ros_exercises-msg:primes_list instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <primes_list>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_exercises-msg:data-val is deprecated.  Use ros_exercises-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <primes_list>) ostream)
  "Serializes a message object of type '<primes_list>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <primes_list>) istream)
  "Deserializes a message object of type '<primes_list>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<primes_list>)))
  "Returns string type for a message object of type '<primes_list>"
  "ros_exercises/primes_list")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'primes_list)))
  "Returns string type for a message object of type 'primes_list"
  "ros_exercises/primes_list")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<primes_list>)))
  "Returns md5sum for a message object of type '<primes_list>"
  "715b81068192546a86f8c2db120d2a3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'primes_list)))
  "Returns md5sum for a message object of type 'primes_list"
  "715b81068192546a86f8c2db120d2a3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<primes_list>)))
  "Returns full string definition for message of type '<primes_list>"
  (cl:format cl:nil "uint64[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'primes_list)))
  "Returns full string definition for message of type 'primes_list"
  (cl:format cl:nil "uint64[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <primes_list>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <primes_list>))
  "Converts a ROS message object to a list"
  (cl:list 'primes_list
    (cl:cons ':data (data msg))
))
