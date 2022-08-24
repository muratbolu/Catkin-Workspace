; Auto-generated. Do not edit!


(cl:in-package ros_exercises-srv)


;//! \htmlinclude check_palindrome-request.msg.html

(cl:defclass <check_palindrome-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type cl:string
    :initform ""))
)

(cl:defclass check_palindrome-request (<check_palindrome-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <check_palindrome-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'check_palindrome-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_exercises-srv:<check_palindrome-request> is deprecated: use ros_exercises-srv:check_palindrome-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <check_palindrome-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_exercises-srv:input-val is deprecated.  Use ros_exercises-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <check_palindrome-request>) ostream)
  "Serializes a message object of type '<check_palindrome-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'input))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'input))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <check_palindrome-request>) istream)
  "Deserializes a message object of type '<check_palindrome-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'input) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'input) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<check_palindrome-request>)))
  "Returns string type for a service object of type '<check_palindrome-request>"
  "ros_exercises/check_palindromeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'check_palindrome-request)))
  "Returns string type for a service object of type 'check_palindrome-request"
  "ros_exercises/check_palindromeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<check_palindrome-request>)))
  "Returns md5sum for a message object of type '<check_palindrome-request>"
  "9435240a40d8ae413a3b7925d764bbcf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'check_palindrome-request)))
  "Returns md5sum for a message object of type 'check_palindrome-request"
  "9435240a40d8ae413a3b7925d764bbcf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<check_palindrome-request>)))
  "Returns full string definition for message of type '<check_palindrome-request>"
  (cl:format cl:nil "string input~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'check_palindrome-request)))
  "Returns full string definition for message of type 'check_palindrome-request"
  (cl:format cl:nil "string input~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <check_palindrome-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <check_palindrome-request>))
  "Converts a ROS message object to a list"
  (cl:list 'check_palindrome-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude check_palindrome-response.msg.html

(cl:defclass <check_palindrome-response> (roslisp-msg-protocol:ros-message)
  ((bit
    :reader bit
    :initarg :bit
    :type cl:fixnum
    :initform 0))
)

(cl:defclass check_palindrome-response (<check_palindrome-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <check_palindrome-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'check_palindrome-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_exercises-srv:<check_palindrome-response> is deprecated: use ros_exercises-srv:check_palindrome-response instead.")))

(cl:ensure-generic-function 'bit-val :lambda-list '(m))
(cl:defmethod bit-val ((m <check_palindrome-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_exercises-srv:bit-val is deprecated.  Use ros_exercises-srv:bit instead.")
  (bit m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <check_palindrome-response>) ostream)
  "Serializes a message object of type '<check_palindrome-response>"
  (cl:let* ((signed (cl:slot-value msg 'bit)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <check_palindrome-response>) istream)
  "Deserializes a message object of type '<check_palindrome-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bit) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<check_palindrome-response>)))
  "Returns string type for a service object of type '<check_palindrome-response>"
  "ros_exercises/check_palindromeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'check_palindrome-response)))
  "Returns string type for a service object of type 'check_palindrome-response"
  "ros_exercises/check_palindromeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<check_palindrome-response>)))
  "Returns md5sum for a message object of type '<check_palindrome-response>"
  "9435240a40d8ae413a3b7925d764bbcf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'check_palindrome-response)))
  "Returns md5sum for a message object of type 'check_palindrome-response"
  "9435240a40d8ae413a3b7925d764bbcf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<check_palindrome-response>)))
  "Returns full string definition for message of type '<check_palindrome-response>"
  (cl:format cl:nil "int8 bit~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'check_palindrome-response)))
  "Returns full string definition for message of type 'check_palindrome-response"
  (cl:format cl:nil "int8 bit~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <check_palindrome-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <check_palindrome-response>))
  "Converts a ROS message object to a list"
  (cl:list 'check_palindrome-response
    (cl:cons ':bit (bit msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'check_palindrome)))
  'check_palindrome-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'check_palindrome)))
  'check_palindrome-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'check_palindrome)))
  "Returns string type for a service object of type '<check_palindrome>"
  "ros_exercises/check_palindrome")