; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ArmStateInformation.msg.html

(cl:defclass <ArmStateInformation> (roslisp-msg-protocol:ros-message)
  ((active_state
    :reader active_state
    :initarg :active_state
    :type cl:integer
    :initform 0)
   (connection
    :reader connection
    :initarg :connection
    :type kortex_driver-msg:Connection
    :initform (cl:make-instance 'kortex_driver-msg:Connection)))
)

(cl:defclass ArmStateInformation (<ArmStateInformation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmStateInformation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmStateInformation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ArmStateInformation> is deprecated: use kortex_driver-msg:ArmStateInformation instead.")))

(cl:ensure-generic-function 'active_state-val :lambda-list '(m))
(cl:defmethod active_state-val ((m <ArmStateInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:active_state-val is deprecated.  Use kortex_driver-msg:active_state instead.")
  (active_state m))

(cl:ensure-generic-function 'connection-val :lambda-list '(m))
(cl:defmethod connection-val ((m <ArmStateInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:connection-val is deprecated.  Use kortex_driver-msg:connection instead.")
  (connection m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmStateInformation>) ostream)
  "Serializes a message object of type '<ArmStateInformation>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'active_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'active_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'active_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'active_state)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'connection) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmStateInformation>) istream)
  "Deserializes a message object of type '<ArmStateInformation>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'active_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'active_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'active_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'active_state)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'connection) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmStateInformation>)))
  "Returns string type for a message object of type '<ArmStateInformation>"
  "kortex_driver/ArmStateInformation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmStateInformation)))
  "Returns string type for a message object of type 'ArmStateInformation"
  "kortex_driver/ArmStateInformation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmStateInformation>)))
  "Returns md5sum for a message object of type '<ArmStateInformation>"
  "dae937682c51c5ef4ccc506f80df3c9f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmStateInformation)))
  "Returns md5sum for a message object of type 'ArmStateInformation"
  "dae937682c51c5ef4ccc506f80df3c9f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmStateInformation>)))
  "Returns full string definition for message of type '<ArmStateInformation>"
  (cl:format cl:nil "~%uint32 active_state~%Connection connection~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmStateInformation)))
  "Returns full string definition for message of type 'ArmStateInformation"
  (cl:format cl:nil "~%uint32 active_state~%Connection connection~%================================================================================~%MSG: kortex_driver/Connection~%~%UserProfileHandle user_handle~%string connection_information~%uint32 connection_identifier~%================================================================================~%MSG: kortex_driver/UserProfileHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmStateInformation>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'connection))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmStateInformation>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmStateInformation
    (cl:cons ':active_state (active_state msg))
    (cl:cons ':connection (connection msg))
))
