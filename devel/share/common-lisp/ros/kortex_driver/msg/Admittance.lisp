; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Admittance.msg.html

(cl:defclass <Admittance> (roslisp-msg-protocol:ros-message)
  ((admittance_mode
    :reader admittance_mode
    :initarg :admittance_mode
    :type cl:integer
    :initform 0))
)

(cl:defclass Admittance (<Admittance>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Admittance>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Admittance)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Admittance> is deprecated: use kortex_driver-msg:Admittance instead.")))

(cl:ensure-generic-function 'admittance_mode-val :lambda-list '(m))
(cl:defmethod admittance_mode-val ((m <Admittance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:admittance_mode-val is deprecated.  Use kortex_driver-msg:admittance_mode instead.")
  (admittance_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Admittance>) ostream)
  "Serializes a message object of type '<Admittance>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'admittance_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'admittance_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'admittance_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'admittance_mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Admittance>) istream)
  "Deserializes a message object of type '<Admittance>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'admittance_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'admittance_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'admittance_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'admittance_mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Admittance>)))
  "Returns string type for a message object of type '<Admittance>"
  "kortex_driver/Admittance")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Admittance)))
  "Returns string type for a message object of type 'Admittance"
  "kortex_driver/Admittance")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Admittance>)))
  "Returns md5sum for a message object of type '<Admittance>"
  "5f17929e02863c928199e8741332cc24")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Admittance)))
  "Returns md5sum for a message object of type 'Admittance"
  "5f17929e02863c928199e8741332cc24")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Admittance>)))
  "Returns full string definition for message of type '<Admittance>"
  (cl:format cl:nil "~%uint32 admittance_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Admittance)))
  "Returns full string definition for message of type 'Admittance"
  (cl:format cl:nil "~%uint32 admittance_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Admittance>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Admittance>))
  "Converts a ROS message object to a list"
  (cl:list 'Admittance
    (cl:cons ':admittance_mode (admittance_mode msg))
))
