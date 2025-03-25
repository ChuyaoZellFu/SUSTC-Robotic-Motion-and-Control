; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SwitchControlMapping.msg.html

(cl:defclass <SwitchControlMapping> (roslisp-msg-protocol:ros-message)
  ((controller_identifier
    :reader controller_identifier
    :initarg :controller_identifier
    :type cl:integer
    :initform 0)
   (map_group_handle
    :reader map_group_handle
    :initarg :map_group_handle
    :type kortex_driver-msg:MapGroupHandle
    :initform (cl:make-instance 'kortex_driver-msg:MapGroupHandle))
   (map_handle
    :reader map_handle
    :initarg :map_handle
    :type kortex_driver-msg:MapHandle
    :initform (cl:make-instance 'kortex_driver-msg:MapHandle)))
)

(cl:defclass SwitchControlMapping (<SwitchControlMapping>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwitchControlMapping>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwitchControlMapping)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SwitchControlMapping> is deprecated: use kortex_driver-msg:SwitchControlMapping instead.")))

(cl:ensure-generic-function 'controller_identifier-val :lambda-list '(m))
(cl:defmethod controller_identifier-val ((m <SwitchControlMapping>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:controller_identifier-val is deprecated.  Use kortex_driver-msg:controller_identifier instead.")
  (controller_identifier m))

(cl:ensure-generic-function 'map_group_handle-val :lambda-list '(m))
(cl:defmethod map_group_handle-val ((m <SwitchControlMapping>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:map_group_handle-val is deprecated.  Use kortex_driver-msg:map_group_handle instead.")
  (map_group_handle m))

(cl:ensure-generic-function 'map_handle-val :lambda-list '(m))
(cl:defmethod map_handle-val ((m <SwitchControlMapping>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:map_handle-val is deprecated.  Use kortex_driver-msg:map_handle instead.")
  (map_handle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwitchControlMapping>) ostream)
  "Serializes a message object of type '<SwitchControlMapping>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'controller_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'controller_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'controller_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'controller_identifier)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'map_group_handle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'map_handle) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwitchControlMapping>) istream)
  "Deserializes a message object of type '<SwitchControlMapping>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'controller_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'controller_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'controller_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'controller_identifier)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'map_group_handle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'map_handle) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwitchControlMapping>)))
  "Returns string type for a message object of type '<SwitchControlMapping>"
  "kortex_driver/SwitchControlMapping")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwitchControlMapping)))
  "Returns string type for a message object of type 'SwitchControlMapping"
  "kortex_driver/SwitchControlMapping")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwitchControlMapping>)))
  "Returns md5sum for a message object of type '<SwitchControlMapping>"
  "4a7c904b5b1279f1ceda6c334529fa90")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwitchControlMapping)))
  "Returns md5sum for a message object of type 'SwitchControlMapping"
  "4a7c904b5b1279f1ceda6c334529fa90")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwitchControlMapping>)))
  "Returns full string definition for message of type '<SwitchControlMapping>"
  (cl:format cl:nil "~%uint32 controller_identifier~%MapGroupHandle map_group_handle~%MapHandle map_handle~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwitchControlMapping)))
  "Returns full string definition for message of type 'SwitchControlMapping"
  (cl:format cl:nil "~%uint32 controller_identifier~%MapGroupHandle map_group_handle~%MapHandle map_handle~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwitchControlMapping>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'map_group_handle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'map_handle))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwitchControlMapping>))
  "Converts a ROS message object to a list"
  (cl:list 'SwitchControlMapping
    (cl:cons ':controller_identifier (controller_identifier msg))
    (cl:cons ':map_group_handle (map_group_handle msg))
    (cl:cons ':map_handle (map_handle msg))
))
