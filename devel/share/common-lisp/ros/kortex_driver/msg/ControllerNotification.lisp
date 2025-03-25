; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControllerNotification.msg.html

(cl:defclass <ControllerNotification> (roslisp-msg-protocol:ros-message)
  ((timestamp
    :reader timestamp
    :initarg :timestamp
    :type kortex_driver-msg:Timestamp
    :initform (cl:make-instance 'kortex_driver-msg:Timestamp))
   (user_handle
    :reader user_handle
    :initarg :user_handle
    :type kortex_driver-msg:UserProfileHandle
    :initform (cl:make-instance 'kortex_driver-msg:UserProfileHandle))
   (connection
    :reader connection
    :initarg :connection
    :type kortex_driver-msg:Connection
    :initform (cl:make-instance 'kortex_driver-msg:Connection))
   (oneof_state
    :reader oneof_state
    :initarg :oneof_state
    :type kortex_driver-msg:ControllerNotification_state
    :initform (cl:make-instance 'kortex_driver-msg:ControllerNotification_state)))
)

(cl:defclass ControllerNotification (<ControllerNotification>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerNotification>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerNotification)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControllerNotification> is deprecated: use kortex_driver-msg:ControllerNotification instead.")))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <ControllerNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:timestamp-val is deprecated.  Use kortex_driver-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'user_handle-val :lambda-list '(m))
(cl:defmethod user_handle-val ((m <ControllerNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:user_handle-val is deprecated.  Use kortex_driver-msg:user_handle instead.")
  (user_handle m))

(cl:ensure-generic-function 'connection-val :lambda-list '(m))
(cl:defmethod connection-val ((m <ControllerNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:connection-val is deprecated.  Use kortex_driver-msg:connection instead.")
  (connection m))

(cl:ensure-generic-function 'oneof_state-val :lambda-list '(m))
(cl:defmethod oneof_state-val ((m <ControllerNotification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:oneof_state-val is deprecated.  Use kortex_driver-msg:oneof_state instead.")
  (oneof_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerNotification>) ostream)
  "Serializes a message object of type '<ControllerNotification>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'timestamp) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'user_handle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'connection) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'oneof_state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerNotification>) istream)
  "Deserializes a message object of type '<ControllerNotification>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'timestamp) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'user_handle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'connection) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'oneof_state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerNotification>)))
  "Returns string type for a message object of type '<ControllerNotification>"
  "kortex_driver/ControllerNotification")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerNotification)))
  "Returns string type for a message object of type 'ControllerNotification"
  "kortex_driver/ControllerNotification")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerNotification>)))
  "Returns md5sum for a message object of type '<ControllerNotification>"
  "4a8a872ea40ec244a141a390ef3880ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerNotification)))
  "Returns md5sum for a message object of type 'ControllerNotification"
  "4a8a872ea40ec244a141a390ef3880ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerNotification>)))
  "Returns full string definition for message of type '<ControllerNotification>"
  (cl:format cl:nil "~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%ControllerNotification_state oneof_state~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/ControllerNotification_state~%~%ControllerState[] controller_state~%ControllerElementState[] controller_element~%================================================================================~%MSG: kortex_driver/ControllerState~%~%ControllerHandle handle~%uint32 event_type~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/ControllerElementState~%~%ControllerElementHandle handle~%uint32 event_type~%float32 axis_value~%================================================================================~%MSG: kortex_driver/ControllerElementHandle~%~%ControllerHandle controller_handle~%ControllerElementHandle_identifier oneof_identifier~%================================================================================~%MSG: kortex_driver/ControllerElementHandle_identifier~%~%uint32[] button~%uint32[] axis~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerNotification)))
  "Returns full string definition for message of type 'ControllerNotification"
  (cl:format cl:nil "~%Timestamp timestamp~%UserProfileHandle user_handle~%Connection connection~%ControllerNotification_state oneof_state~%================================================================================~%MSG: kortex_driver/Timestamp~%~%uint32 sec~%uint32 usec~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/ControllerNotification_state~%~%ControllerState[] controller_state~%ControllerElementState[] controller_element~%================================================================================~%MSG: kortex_driver/ControllerState~%~%ControllerHandle handle~%uint32 event_type~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/ControllerElementState~%~%ControllerElementHandle handle~%uint32 event_type~%float32 axis_value~%================================================================================~%MSG: kortex_driver/ControllerElementHandle~%~%ControllerHandle controller_handle~%ControllerElementHandle_identifier oneof_identifier~%================================================================================~%MSG: kortex_driver/ControllerElementHandle_identifier~%~%uint32[] button~%uint32[] axis~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerNotification>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'timestamp))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'user_handle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'connection))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'oneof_state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerNotification>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerNotification
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':user_handle (user_handle msg))
    (cl:cons ':connection (connection msg))
    (cl:cons ':oneof_state (oneof_state msg))
))
