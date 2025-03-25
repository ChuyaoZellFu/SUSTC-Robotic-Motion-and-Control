; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude PayloadInformation.msg.html

(cl:defclass <PayloadInformation> (roslisp-msg-protocol:ros-message)
  ((payload_mass
    :reader payload_mass
    :initarg :payload_mass
    :type cl:float
    :initform 0.0)
   (payload_mass_center
    :reader payload_mass_center
    :initarg :payload_mass_center
    :type kortex_driver-msg:ControlConfig_Position
    :initform (cl:make-instance 'kortex_driver-msg:ControlConfig_Position)))
)

(cl:defclass PayloadInformation (<PayloadInformation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PayloadInformation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PayloadInformation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<PayloadInformation> is deprecated: use kortex_driver-msg:PayloadInformation instead.")))

(cl:ensure-generic-function 'payload_mass-val :lambda-list '(m))
(cl:defmethod payload_mass-val ((m <PayloadInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:payload_mass-val is deprecated.  Use kortex_driver-msg:payload_mass instead.")
  (payload_mass m))

(cl:ensure-generic-function 'payload_mass_center-val :lambda-list '(m))
(cl:defmethod payload_mass_center-val ((m <PayloadInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:payload_mass_center-val is deprecated.  Use kortex_driver-msg:payload_mass_center instead.")
  (payload_mass_center m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PayloadInformation>) ostream)
  "Serializes a message object of type '<PayloadInformation>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'payload_mass))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'payload_mass_center) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PayloadInformation>) istream)
  "Deserializes a message object of type '<PayloadInformation>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'payload_mass) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'payload_mass_center) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PayloadInformation>)))
  "Returns string type for a message object of type '<PayloadInformation>"
  "kortex_driver/PayloadInformation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PayloadInformation)))
  "Returns string type for a message object of type 'PayloadInformation"
  "kortex_driver/PayloadInformation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PayloadInformation>)))
  "Returns md5sum for a message object of type '<PayloadInformation>"
  "b6414681a38d62f0a2b6ff0a6a813f53")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PayloadInformation)))
  "Returns md5sum for a message object of type 'PayloadInformation"
  "b6414681a38d62f0a2b6ff0a6a813f53")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PayloadInformation>)))
  "Returns full string definition for message of type '<PayloadInformation>"
  (cl:format cl:nil "~%float32 payload_mass~%ControlConfig_Position payload_mass_center~%================================================================================~%MSG: kortex_driver/ControlConfig_Position~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PayloadInformation)))
  "Returns full string definition for message of type 'PayloadInformation"
  (cl:format cl:nil "~%float32 payload_mass~%ControlConfig_Position payload_mass_center~%================================================================================~%MSG: kortex_driver/ControlConfig_Position~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PayloadInformation>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'payload_mass_center))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PayloadInformation>))
  "Converts a ROS message object to a list"
  (cl:list 'PayloadInformation
    (cl:cons ':payload_mass (payload_mass msg))
    (cl:cons ':payload_mass_center (payload_mass_center msg))
))
