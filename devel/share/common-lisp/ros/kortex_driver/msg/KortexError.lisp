; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude KortexError.msg.html

(cl:defclass <KortexError> (roslisp-msg-protocol:ros-message)
  ((description
    :reader description
    :initarg :description
    :type cl:string
    :initform "")
   (code
    :reader code
    :initarg :code
    :type cl:integer
    :initform 0)
   (subCode
    :reader subCode
    :initarg :subCode
    :type cl:integer
    :initform 0))
)

(cl:defclass KortexError (<KortexError>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KortexError>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KortexError)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<KortexError> is deprecated: use kortex_driver-msg:KortexError instead.")))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <KortexError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:description-val is deprecated.  Use kortex_driver-msg:description instead.")
  (description m))

(cl:ensure-generic-function 'code-val :lambda-list '(m))
(cl:defmethod code-val ((m <KortexError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:code-val is deprecated.  Use kortex_driver-msg:code instead.")
  (code m))

(cl:ensure-generic-function 'subCode-val :lambda-list '(m))
(cl:defmethod subCode-val ((m <KortexError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:subCode-val is deprecated.  Use kortex_driver-msg:subCode instead.")
  (subCode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KortexError>) ostream)
  "Serializes a message object of type '<KortexError>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'description))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'code)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'code)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'code)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'code)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'subCode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'subCode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'subCode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'subCode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KortexError>) istream)
  "Deserializes a message object of type '<KortexError>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'code)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'code)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'code)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'code)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'subCode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'subCode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'subCode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'subCode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KortexError>)))
  "Returns string type for a message object of type '<KortexError>"
  "kortex_driver/KortexError")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KortexError)))
  "Returns string type for a message object of type 'KortexError"
  "kortex_driver/KortexError")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KortexError>)))
  "Returns md5sum for a message object of type '<KortexError>"
  "e30b4b6c4d6da8786415fb734a7dea1b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KortexError)))
  "Returns md5sum for a message object of type 'KortexError"
  "e30b4b6c4d6da8786415fb734a7dea1b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KortexError>)))
  "Returns full string definition for message of type '<KortexError>"
  (cl:format cl:nil "string description~%uint32 code~%uint32 subCode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KortexError)))
  "Returns full string definition for message of type 'KortexError"
  (cl:format cl:nil "string description~%uint32 code~%uint32 subCode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KortexError>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'description))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KortexError>))
  "Converts a ROS message object to a list"
  (cl:list 'KortexError
    (cl:cons ':description (description msg))
    (cl:cons ':code (code msg))
    (cl:cons ':subCode (subCode msg))
))
