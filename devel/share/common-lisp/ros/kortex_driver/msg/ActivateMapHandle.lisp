; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ActivateMapHandle.msg.html

(cl:defclass <ActivateMapHandle> (roslisp-msg-protocol:ros-message)
  ((mapping_handle
    :reader mapping_handle
    :initarg :mapping_handle
    :type kortex_driver-msg:MappingHandle
    :initform (cl:make-instance 'kortex_driver-msg:MappingHandle))
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

(cl:defclass ActivateMapHandle (<ActivateMapHandle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActivateMapHandle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActivateMapHandle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ActivateMapHandle> is deprecated: use kortex_driver-msg:ActivateMapHandle instead.")))

(cl:ensure-generic-function 'mapping_handle-val :lambda-list '(m))
(cl:defmethod mapping_handle-val ((m <ActivateMapHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:mapping_handle-val is deprecated.  Use kortex_driver-msg:mapping_handle instead.")
  (mapping_handle m))

(cl:ensure-generic-function 'map_group_handle-val :lambda-list '(m))
(cl:defmethod map_group_handle-val ((m <ActivateMapHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:map_group_handle-val is deprecated.  Use kortex_driver-msg:map_group_handle instead.")
  (map_group_handle m))

(cl:ensure-generic-function 'map_handle-val :lambda-list '(m))
(cl:defmethod map_handle-val ((m <ActivateMapHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:map_handle-val is deprecated.  Use kortex_driver-msg:map_handle instead.")
  (map_handle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActivateMapHandle>) ostream)
  "Serializes a message object of type '<ActivateMapHandle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mapping_handle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'map_group_handle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'map_handle) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActivateMapHandle>) istream)
  "Deserializes a message object of type '<ActivateMapHandle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mapping_handle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'map_group_handle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'map_handle) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActivateMapHandle>)))
  "Returns string type for a message object of type '<ActivateMapHandle>"
  "kortex_driver/ActivateMapHandle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActivateMapHandle)))
  "Returns string type for a message object of type 'ActivateMapHandle"
  "kortex_driver/ActivateMapHandle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActivateMapHandle>)))
  "Returns md5sum for a message object of type '<ActivateMapHandle>"
  "3f76b24edc6fa7ed6cb77b6c5ee77745")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActivateMapHandle)))
  "Returns md5sum for a message object of type 'ActivateMapHandle"
  "3f76b24edc6fa7ed6cb77b6c5ee77745")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActivateMapHandle>)))
  "Returns full string definition for message of type '<ActivateMapHandle>"
  (cl:format cl:nil "~%MappingHandle mapping_handle~%MapGroupHandle map_group_handle~%MapHandle map_handle~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActivateMapHandle)))
  "Returns full string definition for message of type 'ActivateMapHandle"
  (cl:format cl:nil "~%MappingHandle mapping_handle~%MapGroupHandle map_group_handle~%MapHandle map_handle~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActivateMapHandle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mapping_handle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'map_group_handle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'map_handle))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActivateMapHandle>))
  "Converts a ROS message object to a list"
  (cl:list 'ActivateMapHandle
    (cl:cons ':mapping_handle (mapping_handle msg))
    (cl:cons ':map_group_handle (map_group_handle msg))
    (cl:cons ':map_handle (map_handle msg))
))
