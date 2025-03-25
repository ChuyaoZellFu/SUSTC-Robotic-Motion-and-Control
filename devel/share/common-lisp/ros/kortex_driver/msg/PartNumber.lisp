; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude PartNumber.msg.html

(cl:defclass <PartNumber> (roslisp-msg-protocol:ros-message)
  ((part_number
    :reader part_number
    :initarg :part_number
    :type cl:string
    :initform ""))
)

(cl:defclass PartNumber (<PartNumber>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PartNumber>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PartNumber)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<PartNumber> is deprecated: use kortex_driver-msg:PartNumber instead.")))

(cl:ensure-generic-function 'part_number-val :lambda-list '(m))
(cl:defmethod part_number-val ((m <PartNumber>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:part_number-val is deprecated.  Use kortex_driver-msg:part_number instead.")
  (part_number m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PartNumber>) ostream)
  "Serializes a message object of type '<PartNumber>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'part_number))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'part_number))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PartNumber>) istream)
  "Deserializes a message object of type '<PartNumber>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'part_number) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'part_number) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PartNumber>)))
  "Returns string type for a message object of type '<PartNumber>"
  "kortex_driver/PartNumber")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PartNumber)))
  "Returns string type for a message object of type 'PartNumber"
  "kortex_driver/PartNumber")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PartNumber>)))
  "Returns md5sum for a message object of type '<PartNumber>"
  "ce9f25be628512aac32b499f98199661")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PartNumber)))
  "Returns md5sum for a message object of type 'PartNumber"
  "ce9f25be628512aac32b499f98199661")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PartNumber>)))
  "Returns full string definition for message of type '<PartNumber>"
  (cl:format cl:nil "~%string part_number~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PartNumber)))
  "Returns full string definition for message of type 'PartNumber"
  (cl:format cl:nil "~%string part_number~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PartNumber>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'part_number))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PartNumber>))
  "Converts a ROS message object to a list"
  (cl:list 'PartNumber
    (cl:cons ':part_number (part_number msg))
))
