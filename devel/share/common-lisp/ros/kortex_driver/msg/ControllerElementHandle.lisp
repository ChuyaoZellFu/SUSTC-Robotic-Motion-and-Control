; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControllerElementHandle.msg.html

(cl:defclass <ControllerElementHandle> (roslisp-msg-protocol:ros-message)
  ((controller_handle
    :reader controller_handle
    :initarg :controller_handle
    :type kortex_driver-msg:ControllerHandle
    :initform (cl:make-instance 'kortex_driver-msg:ControllerHandle))
   (oneof_identifier
    :reader oneof_identifier
    :initarg :oneof_identifier
    :type kortex_driver-msg:ControllerElementHandle_identifier
    :initform (cl:make-instance 'kortex_driver-msg:ControllerElementHandle_identifier)))
)

(cl:defclass ControllerElementHandle (<ControllerElementHandle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerElementHandle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerElementHandle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControllerElementHandle> is deprecated: use kortex_driver-msg:ControllerElementHandle instead.")))

(cl:ensure-generic-function 'controller_handle-val :lambda-list '(m))
(cl:defmethod controller_handle-val ((m <ControllerElementHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:controller_handle-val is deprecated.  Use kortex_driver-msg:controller_handle instead.")
  (controller_handle m))

(cl:ensure-generic-function 'oneof_identifier-val :lambda-list '(m))
(cl:defmethod oneof_identifier-val ((m <ControllerElementHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:oneof_identifier-val is deprecated.  Use kortex_driver-msg:oneof_identifier instead.")
  (oneof_identifier m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerElementHandle>) ostream)
  "Serializes a message object of type '<ControllerElementHandle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'controller_handle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'oneof_identifier) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerElementHandle>) istream)
  "Deserializes a message object of type '<ControllerElementHandle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'controller_handle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'oneof_identifier) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerElementHandle>)))
  "Returns string type for a message object of type '<ControllerElementHandle>"
  "kortex_driver/ControllerElementHandle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerElementHandle)))
  "Returns string type for a message object of type 'ControllerElementHandle"
  "kortex_driver/ControllerElementHandle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerElementHandle>)))
  "Returns md5sum for a message object of type '<ControllerElementHandle>"
  "8c89bd366e7348e92cb61f477829e5ba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerElementHandle)))
  "Returns md5sum for a message object of type 'ControllerElementHandle"
  "8c89bd366e7348e92cb61f477829e5ba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerElementHandle>)))
  "Returns full string definition for message of type '<ControllerElementHandle>"
  (cl:format cl:nil "~%ControllerHandle controller_handle~%ControllerElementHandle_identifier oneof_identifier~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/ControllerElementHandle_identifier~%~%uint32[] button~%uint32[] axis~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerElementHandle)))
  "Returns full string definition for message of type 'ControllerElementHandle"
  (cl:format cl:nil "~%ControllerHandle controller_handle~%ControllerElementHandle_identifier oneof_identifier~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/ControllerElementHandle_identifier~%~%uint32[] button~%uint32[] axis~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerElementHandle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'controller_handle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'oneof_identifier))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerElementHandle>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerElementHandle
    (cl:cons ':controller_handle (controller_handle msg))
    (cl:cons ':oneof_identifier (oneof_identifier msg))
))
