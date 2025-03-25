; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude MapHandle.msg.html

(cl:defclass <MapHandle> (roslisp-msg-protocol:ros-message)
  ((identifier
    :reader identifier
    :initarg :identifier
    :type cl:integer
    :initform 0)
   (permission
    :reader permission
    :initarg :permission
    :type cl:integer
    :initform 0))
)

(cl:defclass MapHandle (<MapHandle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapHandle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapHandle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<MapHandle> is deprecated: use kortex_driver-msg:MapHandle instead.")))

(cl:ensure-generic-function 'identifier-val :lambda-list '(m))
(cl:defmethod identifier-val ((m <MapHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:identifier-val is deprecated.  Use kortex_driver-msg:identifier instead.")
  (identifier m))

(cl:ensure-generic-function 'permission-val :lambda-list '(m))
(cl:defmethod permission-val ((m <MapHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:permission-val is deprecated.  Use kortex_driver-msg:permission instead.")
  (permission m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapHandle>) ostream)
  "Serializes a message object of type '<MapHandle>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'permission)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'permission)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'permission)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'permission)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapHandle>) istream)
  "Deserializes a message object of type '<MapHandle>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'permission)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'permission)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'permission)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'permission)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapHandle>)))
  "Returns string type for a message object of type '<MapHandle>"
  "kortex_driver/MapHandle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapHandle)))
  "Returns string type for a message object of type 'MapHandle"
  "kortex_driver/MapHandle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapHandle>)))
  "Returns md5sum for a message object of type '<MapHandle>"
  "10d841c6abc3fd3596d9b10510d50074")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapHandle)))
  "Returns md5sum for a message object of type 'MapHandle"
  "10d841c6abc3fd3596d9b10510d50074")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapHandle>)))
  "Returns full string definition for message of type '<MapHandle>"
  (cl:format cl:nil "~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapHandle)))
  "Returns full string definition for message of type 'MapHandle"
  (cl:format cl:nil "~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapHandle>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapHandle>))
  "Converts a ROS message object to a list"
  (cl:list 'MapHandle
    (cl:cons ':identifier (identifier msg))
    (cl:cons ':permission (permission msg))
))
