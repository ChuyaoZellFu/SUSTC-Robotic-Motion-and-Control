; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ModelNumber.msg.html

(cl:defclass <ModelNumber> (roslisp-msg-protocol:ros-message)
  ((model_number
    :reader model_number
    :initarg :model_number
    :type cl:string
    :initform ""))
)

(cl:defclass ModelNumber (<ModelNumber>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModelNumber>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModelNumber)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ModelNumber> is deprecated: use kortex_driver-msg:ModelNumber instead.")))

(cl:ensure-generic-function 'model_number-val :lambda-list '(m))
(cl:defmethod model_number-val ((m <ModelNumber>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:model_number-val is deprecated.  Use kortex_driver-msg:model_number instead.")
  (model_number m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModelNumber>) ostream)
  "Serializes a message object of type '<ModelNumber>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model_number))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model_number))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModelNumber>) istream)
  "Deserializes a message object of type '<ModelNumber>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model_number) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model_number) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModelNumber>)))
  "Returns string type for a message object of type '<ModelNumber>"
  "kortex_driver/ModelNumber")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModelNumber)))
  "Returns string type for a message object of type 'ModelNumber"
  "kortex_driver/ModelNumber")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModelNumber>)))
  "Returns md5sum for a message object of type '<ModelNumber>"
  "f12cb4b23059487b453a88e8d41162b5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModelNumber)))
  "Returns md5sum for a message object of type 'ModelNumber"
  "f12cb4b23059487b453a88e8d41162b5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModelNumber>)))
  "Returns full string definition for message of type '<ModelNumber>"
  (cl:format cl:nil "~%string model_number~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModelNumber)))
  "Returns full string definition for message of type 'ModelNumber"
  (cl:format cl:nil "~%string model_number~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModelNumber>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'model_number))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModelNumber>))
  "Converts a ROS message object to a list"
  (cl:list 'ModelNumber
    (cl:cons ':model_number (model_number msg))
))
