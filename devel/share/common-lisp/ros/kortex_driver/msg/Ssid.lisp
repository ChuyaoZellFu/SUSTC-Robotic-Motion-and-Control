; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Ssid.msg.html

(cl:defclass <Ssid> (roslisp-msg-protocol:ros-message)
  ((identifier
    :reader identifier
    :initarg :identifier
    :type cl:string
    :initform ""))
)

(cl:defclass Ssid (<Ssid>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Ssid>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Ssid)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Ssid> is deprecated: use kortex_driver-msg:Ssid instead.")))

(cl:ensure-generic-function 'identifier-val :lambda-list '(m))
(cl:defmethod identifier-val ((m <Ssid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:identifier-val is deprecated.  Use kortex_driver-msg:identifier instead.")
  (identifier m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Ssid>) ostream)
  "Serializes a message object of type '<Ssid>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'identifier))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'identifier))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Ssid>) istream)
  "Deserializes a message object of type '<Ssid>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'identifier) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'identifier) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Ssid>)))
  "Returns string type for a message object of type '<Ssid>"
  "kortex_driver/Ssid")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Ssid)))
  "Returns string type for a message object of type 'Ssid"
  "kortex_driver/Ssid")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Ssid>)))
  "Returns md5sum for a message object of type '<Ssid>"
  "9c4b0f781baa7fd49cc9e186f2f56898")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Ssid)))
  "Returns md5sum for a message object of type 'Ssid"
  "9c4b0f781baa7fd49cc9e186f2f56898")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Ssid>)))
  "Returns full string definition for message of type '<Ssid>"
  (cl:format cl:nil "~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Ssid)))
  "Returns full string definition for message of type 'Ssid"
  (cl:format cl:nil "~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Ssid>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'identifier))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Ssid>))
  "Converts a ROS message object to a list"
  (cl:list 'Ssid
    (cl:cons ':identifier (identifier msg))
))
