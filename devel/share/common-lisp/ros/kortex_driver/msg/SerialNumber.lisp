; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SerialNumber.msg.html

(cl:defclass <SerialNumber> (roslisp-msg-protocol:ros-message)
  ((serial_number
    :reader serial_number
    :initarg :serial_number
    :type cl:string
    :initform ""))
)

(cl:defclass SerialNumber (<SerialNumber>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SerialNumber>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SerialNumber)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SerialNumber> is deprecated: use kortex_driver-msg:SerialNumber instead.")))

(cl:ensure-generic-function 'serial_number-val :lambda-list '(m))
(cl:defmethod serial_number-val ((m <SerialNumber>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:serial_number-val is deprecated.  Use kortex_driver-msg:serial_number instead.")
  (serial_number m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SerialNumber>) ostream)
  "Serializes a message object of type '<SerialNumber>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'serial_number))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'serial_number))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SerialNumber>) istream)
  "Deserializes a message object of type '<SerialNumber>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'serial_number) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'serial_number) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SerialNumber>)))
  "Returns string type for a message object of type '<SerialNumber>"
  "kortex_driver/SerialNumber")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SerialNumber)))
  "Returns string type for a message object of type 'SerialNumber"
  "kortex_driver/SerialNumber")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SerialNumber>)))
  "Returns md5sum for a message object of type '<SerialNumber>"
  "fb68edee72c8891f2105e89acba723ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SerialNumber)))
  "Returns md5sum for a message object of type 'SerialNumber"
  "fb68edee72c8891f2105e89acba723ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SerialNumber>)))
  "Returns full string definition for message of type '<SerialNumber>"
  (cl:format cl:nil "~%string serial_number~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SerialNumber)))
  "Returns full string definition for message of type 'SerialNumber"
  (cl:format cl:nil "~%string serial_number~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SerialNumber>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'serial_number))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SerialNumber>))
  "Converts a ROS message object to a list"
  (cl:list 'SerialNumber
    (cl:cons ':serial_number (serial_number msg))
))
