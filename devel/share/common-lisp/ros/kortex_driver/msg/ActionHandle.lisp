; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ActionHandle.msg.html

(cl:defclass <ActionHandle> (roslisp-msg-protocol:ros-message)
  ((identifier
    :reader identifier
    :initarg :identifier
    :type cl:integer
    :initform 0)
   (action_type
    :reader action_type
    :initarg :action_type
    :type cl:integer
    :initform 0)
   (permission
    :reader permission
    :initarg :permission
    :type cl:integer
    :initform 0))
)

(cl:defclass ActionHandle (<ActionHandle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActionHandle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActionHandle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ActionHandle> is deprecated: use kortex_driver-msg:ActionHandle instead.")))

(cl:ensure-generic-function 'identifier-val :lambda-list '(m))
(cl:defmethod identifier-val ((m <ActionHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:identifier-val is deprecated.  Use kortex_driver-msg:identifier instead.")
  (identifier m))

(cl:ensure-generic-function 'action_type-val :lambda-list '(m))
(cl:defmethod action_type-val ((m <ActionHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:action_type-val is deprecated.  Use kortex_driver-msg:action_type instead.")
  (action_type m))

(cl:ensure-generic-function 'permission-val :lambda-list '(m))
(cl:defmethod permission-val ((m <ActionHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:permission-val is deprecated.  Use kortex_driver-msg:permission instead.")
  (permission m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActionHandle>) ostream)
  "Serializes a message object of type '<ActionHandle>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'action_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'action_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'action_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'action_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'permission)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'permission)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'permission)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'permission)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActionHandle>) istream)
  "Deserializes a message object of type '<ActionHandle>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'action_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'action_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'action_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'action_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'permission)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'permission)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'permission)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'permission)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActionHandle>)))
  "Returns string type for a message object of type '<ActionHandle>"
  "kortex_driver/ActionHandle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActionHandle)))
  "Returns string type for a message object of type 'ActionHandle"
  "kortex_driver/ActionHandle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActionHandle>)))
  "Returns md5sum for a message object of type '<ActionHandle>"
  "107d90c96b3183e90ce57fabb63c6f1f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActionHandle)))
  "Returns md5sum for a message object of type 'ActionHandle"
  "107d90c96b3183e90ce57fabb63c6f1f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActionHandle>)))
  "Returns full string definition for message of type '<ActionHandle>"
  (cl:format cl:nil "~%uint32 identifier~%uint32 action_type~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActionHandle)))
  "Returns full string definition for message of type 'ActionHandle"
  (cl:format cl:nil "~%uint32 identifier~%uint32 action_type~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActionHandle>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActionHandle>))
  "Converts a ROS message object to a list"
  (cl:list 'ActionHandle
    (cl:cons ':identifier (identifier msg))
    (cl:cons ':action_type (action_type msg))
    (cl:cons ':permission (permission msg))
))
