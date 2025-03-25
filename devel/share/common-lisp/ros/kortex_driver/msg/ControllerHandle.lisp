; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControllerHandle.msg.html

(cl:defclass <ControllerHandle> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (controller_identifier
    :reader controller_identifier
    :initarg :controller_identifier
    :type cl:integer
    :initform 0))
)

(cl:defclass ControllerHandle (<ControllerHandle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerHandle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerHandle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControllerHandle> is deprecated: use kortex_driver-msg:ControllerHandle instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <ControllerHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:type-val is deprecated.  Use kortex_driver-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'controller_identifier-val :lambda-list '(m))
(cl:defmethod controller_identifier-val ((m <ControllerHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:controller_identifier-val is deprecated.  Use kortex_driver-msg:controller_identifier instead.")
  (controller_identifier m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerHandle>) ostream)
  "Serializes a message object of type '<ControllerHandle>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'controller_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'controller_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'controller_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'controller_identifier)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerHandle>) istream)
  "Deserializes a message object of type '<ControllerHandle>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'controller_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'controller_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'controller_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'controller_identifier)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerHandle>)))
  "Returns string type for a message object of type '<ControllerHandle>"
  "kortex_driver/ControllerHandle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerHandle)))
  "Returns string type for a message object of type 'ControllerHandle"
  "kortex_driver/ControllerHandle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerHandle>)))
  "Returns md5sum for a message object of type '<ControllerHandle>"
  "e80ce64bff64a2583fb879d270960edd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerHandle)))
  "Returns md5sum for a message object of type 'ControllerHandle"
  "e80ce64bff64a2583fb879d270960edd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerHandle>)))
  "Returns full string definition for message of type '<ControllerHandle>"
  (cl:format cl:nil "~%uint32 type~%uint32 controller_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerHandle)))
  "Returns full string definition for message of type 'ControllerHandle"
  (cl:format cl:nil "~%uint32 type~%uint32 controller_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerHandle>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerHandle>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerHandle
    (cl:cons ':type (type msg))
    (cl:cons ':controller_identifier (controller_identifier msg))
))
